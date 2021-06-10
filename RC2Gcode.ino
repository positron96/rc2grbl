#define USB Serial
#define GRBL Serial1

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

constexpr int N_AXES = 3;

int pins[N_AXES] = { 2,3,4 };

volatile uint32_t lengths[N_AXES];
volatile uint32_t startTimes[N_AXES];

float ofsX, ofsY, ofsZ, x,y,z;
String status;

constexpr int MAX_GCODES = 30;
String gcodes[MAX_GCODES];
int wgcode=0;
int rgcode=0;
bool playMode=false;

bool startsWith(const char *str, const char *pre);

ISR (PCINT0_vect) {
  uint8_t newv = ( PINB & 0b01110000 )>>4;
  
  uint32_t t = micros();
  static uint8_t oldv;
  uint8_t changed = newv ^ oldv;
  oldv = newv;
  
  for(uint8_t i=0; i<N_AXES; i++) {
    if( changed & (1<<i) ) {  // pin i changed
      if( newv & (1<<i) ) { // rising
        startTimes[i] = t;
      } else { // falling
        lengths[i] = t-startTimes[i];
      }
    }
  }
}


void setup() {
  USB.begin(115200);  // usb
  while (!USB) {}
  USB.println("; test!");

  GRBL.begin(115200); // hw  

  // https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328
  cli();
  PCICR |= (1 << PCIE0); // enable PCMSK0
  PCMSK0 = (1<<PCINT4) | (1<<PCINT5) | (1<<PCINT6); // D8, D9, D10
  sei();
  delay(25);

}

constexpr int CENTER = 1500;
constexpr int DEADZONE = 50;

constexpr int MAX_FEEDRATE = 4000; // mm/min, divide by 60 to get mm/s
constexpr int INTL_MS = 15;

static uint32_t lastSent=0;
static bool canSend = true;
  
void loop() {

  bool move = false;
  float vals[N_AXES];
  uint32_t t0 = micros();
  for(int i=0; i<N_AXES; i++) {
    int t = lengths[i];
    int32_t d = t0-startTimes[i];
    if(d > 50000L) { t = CENTER; }
    
    //USB.print(lengths[i]);
    //USB.print(',');
    
    t -= CENTER;
    if(t>DEADZONE) { t-=DEADZONE; } 
    else if(t<-DEADZONE) { t+=DEADZONE; }
    else {t=0;} 
    float v = (float)t/(1000.0-DEADZONE*2)*2;    // -1..1
    if(v!=0) move=true;
    vals[i] = v;

    //USB.print(v);
    //USB.print(',');
  }
  //USB.println();

  if(move) playMode=false;
  
  static bool stopped = false;

  static bool requestStatus = true;
  static uint32_t lastStatusReq=0;

  if(!playMode) {
    if(move) {
      if(millis()-lastSent>INTL_MS) {
        float maxvec = max( abs(vals[0]), abs(vals[1]) );
        maxvec = max(maxvec, abs(vals[2]));
        float feed = maxvec*MAX_FEEDRATE;
        double stepSize = feed/60 * (INTL_MS/1000.0) * 1.1;
        String s="$J=G91 ";
        s += 'X'+String(vals[0]/maxvec*stepSize);
        s += 'Y'+String(vals[1]/maxvec*stepSize);
        s += 'Z'+String(vals[2]/maxvec*stepSize);
        s += 'F'+String(feed);
        
        //String s = String(vals[0])+','+
        //  String(vals[1])+','+
        //  String(vals[2]);
        if(canSend) {
          sendGCode(s);
          stopped = false; //new run
        } 
      }
    } else {
      if(!stopped) {
        GRBL.print('\x85');
        USB.println("TX:!");
        stopped = true;
        lastStatusReq = millis()+200;
        requestStatus = true;
        //canSend = true;
      }
    }
  } else {
    if(canSend && rgcode<wgcode) {
      USB<<"Replaying "<<gcodes[rgcode]<<'\n';
      sendGCode(gcodes[rgcode]);
      rgcode++;
      if(rgcode==wgcode) {playMode=false;}
    }
  }

  if(requestStatus && millis()-lastStatusReq > 1000) {
    GRBL.print('?');
    lastStatusReq = millis();
  }

  static String resp;
  while(GRBL.available()>0) {    
    int t = GRBL.read();
    if(t<0) continue;
    if( t=='\n' || t=='\r' ) {
      if(resp.length()>0) {
        USB.print("RX:");
        USB.println(resp);
        if(resp.charAt(0)=='<') {
          // parse INFO line
          parseGrblStatus(resp.c_str()+1);
          if(status=="Idle") {
            requestStatus = false;
          }
        }
      }
      if(resp=="ok" || startsWith(resp.c_str(), "error:")) {
        canSend = true;
      }
      resp = "";
    } else {
      resp += (char)t;
    }
  }

  while(USB.available()>0) { 
    int t = USB.read();
    switch(t) {
      case 'w': break;
      case '+': { 
        addGCode(String("G1 G90 F1500 X")+x+"Y"+y+"Z"+z);
        break;
      }
      case 'g': {
        sendGCode("M3 S0");
        addGCode("G4 P1"); 
        addGCode("M3 S0"); 
        addGCode("G4 P1"); 
        break;
      }
      case 'r': {
        sendGCode("M3 S500");
        addGCode("G4 P1"); 
        addGCode("M3 S500");
        break;
      }
      case 'x': wgcode=0; break;
      case 'l': {
        USB<<"Listing gcode queue:\n";
        for(int i=0; i<wgcode; i++) USB<<gcodes[i]<<'\n'; 
        break;
      }
      case 'p':
        rgcode=0;
        playMode=true;
        break;
      default: 
        GRBL.write(t);
        break;
    }
  }
}

void sendGCode(String c) {
  GRBL<<c<<'\n';
  USB<<"TX:"<<c<<'\n';
  lastSent = millis();
  canSend = false; // wait for confirmation
}

bool addGCode(String c) {
  if(wgcode>=MAX_GCODES) {USB<<"Limit reached\n"; return false;}
  gcodes[wgcode]=c; 
  wgcode++; 
  return true;
}

bool startsWith(const char *str, const char *pre) {
    return strncmp(pre, str, strlen(pre)) == 0;
}

void mystrcpy(char* dst, const char* start, const char* end) {
    while(start!=end) {
        *(dst++) = *(start++);
    }
    *dst=0;
}

void parseGrblStatus(char* v) {
    //<Idle|MPos:9.800,0.000,0.000|FS:0,0|WCO:0.000,0.000,0.000>
    //<Idle|MPos:9.800,0.000,0.000|FS:0,0|Ov:100,100,100>
    //GD_DEBUGF("parsing %s\n", v.c_str() );
    
    char buf[10];
    bool mpos;
    char cpy[100];
    strncpy(cpy, v, 100);
    v=cpy;

    // idle/jogging
    char* pch = strtok(v, "|");
    if(pch==nullptr) return;
    status = pch; 
    //GD_DEBUGF("Parsed Status: %s\n", status.c_str() );
    USB<<"Parsed status:"<<status<<'\n';

    // MPos:0.000,0.000,0.000
    pch = strtok(nullptr, "|"); 
    if(pch==nullptr) return;
    
    char *st, *fi;
    st=pch+5;fi = strchr(st, ',');   mystrcpy(buf, st, fi);  x = atof(buf);
    st=fi+1; fi = strchr(st, ',');   mystrcpy(buf, st, fi);  y = atof(buf);
    st=fi+1;                                                 z = atof(st);
    mpos = startsWith(pch, "MPos");
    //GD_DEBUGF("Parsed Pos: %f %f %f\n", x,y,z);
    USB<<"Parsed Pos:"<<x<<" "<<y<<" "<<z<<'\n';

    // FS:500,8000 or F:500    
    pch = strtok(nullptr, "|"); 
    while(pch!=nullptr) {
    
        if( startsWith(pch, "FS:") || startsWith(pch, "F:")) {
            /*
            if(pch[1] == 'S') {
                st=pch+3; fi = strchr(st, ','); mystrcpy(buf, st, fi);  feed = atoi(buf);
                st=fi+1;  spindleVal = atoi(st);
            } else {
                feed = atoi(pch+2);
            }
            */
        } else 
        if(startsWith(pch, "WCO:")) {
            st=pch+4;fi = strchr(st, ',');   mystrcpy(buf, st, fi);  ofsX = atof(buf);
            st=fi+1; fi = strchr(st, ',');   mystrcpy(buf, st, fi);  ofsY = atof(buf);
            st=fi+1;                                                 ofsZ = atof(st);
            //GD_DEBUGF("Parsed WCO: %f %f %f\n", ofsX, ofsY, ofsZ);
            USB<<"Parsed WCO:"<<ofsX<<" "<<ofsY<<" "<<ofsZ<<'\n';
        }

        pch = strtok(nullptr, "|"); 

    }
    
    if(!mpos) {
        x -= ofsX; y -= ofsY; z -= ofsZ;
    }

}
