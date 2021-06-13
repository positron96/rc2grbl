#include <PsxControllerHwSpi.h>

#define USB Serial
#define GRBL Serial1

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
template<class T> inline Print &operator <<=(Print &obj, T arg) { obj.println(arg); return obj; }

constexpr int N_AXES = 3;

const byte PIN_PS2_ATT = 10;
PsxControllerHwSpi<PIN_PS2_ATT> psx;

float ofsX, ofsY, ofsZ, x,y,z;
String status;

constexpr int MAX_GCODES = 30;
String gcodes[MAX_GCODES];
int wgcode=0;
int rgcode=0;
bool playMode=false;
bool playCycle=false;

bool startsWith(const char *str, const char *pre);

void hang(){
  while(1){
    pinMode(LED_BUILTIN_TX,INPUT); delay(250);
    pinMode(LED_BUILTIN_TX,OUTPUT);delay(250);
  }
}

void setup() {
  USB.begin(115200); 
  //while (!USB) {}
  USB.println("; test!");

  GRBL.begin(115200); // hw  

  delay(100);
  
  if(!psx.begin() ) { USB<<="No controller"; hang(); }
  if(!psx.enterConfigMode() ) { USB<<="enterConfigMode failed"; hang(); }  
  PsxControllerType ctype = psx.getControllerType();
  USB<<"Type: "<<int(ctype)<<'\n';
  if (!psx.enableAnalogSticks ()) { USB<<="Cannot enable analog sticks"; }
  if (!psx.enableAnalogButtons ()) { USB<<="Cannot enable analog buttons"; }
  psx.exitConfigMode();  
  delay(300);  
  USB.println("Controller ready!");
  
  
}


constexpr int MAX_FEEDRATE = 4000; // mm/min, divide by 60 to get mm/s
constexpr int INTL_MS = 15;

static uint32_t lastSent=0;
static bool canSend = true;
  
void loop() {

  bool move = false;
  float vals[N_AXES];

  static PsxButtons lastBtns=0;
  static uint32_t lastPsx;
  if(millis()-lastPsx>50) {
    lastPsx = millis();
    if(psx.read() ) {
  
      PsxButtons btns = psx.getButtonWord(), t; t=btns;
      btns = (btns ^ lastBtns) & btns;
      lastBtns = t;
     
      if(btns & PSB_CROSS) { 
        addGCode(String("G1 G90 F2000 X")+x+"Y"+y+"Z"+z); 
        USB<<="Added command";
      }
      if(btns & PSB_START) { rgcode=0; playMode=true; }
      if(btns & PSB_SELECT) { 
          USB<<="=== Listing:"; for(int i=0; i<wgcode; i++) USB<<=gcodes[i];
          USB<<="===";
      }
      if(btns & PSB_SQUARE) { 
          USB<<="Cleared program";
          wgcode=0; 
      }
      if(btns & PSB_TRIANGLE) { sendGCode("$H");  }
      if(btns & PSB_L1) {
          GRBL.print('\x84');//door
          playMode = false;
          USB.println("TX:[door]"); 
      }
      if(btns & PSB_R1) {
          GRBL.print('~');
          USB.println("TX:[continue]"); 
      }
      if(btns & PSB_R2) { 
          sendGCode("M3 S0");
          addGCode("G4 P1"); 
          addGCode("M3 S0"); 
          addGCode("G4 P1");     
      }
      if(btns & PSB_L2) { 
          sendGCode("M3 S500");
          addGCode("G4 P1"); 
          addGCode("M3 S500");
      }
      if(btns & PSB_PAD_UP)   { playCycle=true;   }
      if(btns & PSB_PAD_DOWN) { playCycle=false;  }
      
      byte lx, ly;
      psx.getLeftAnalog(lx, ly);
      byte rx, ry;
      psx.getRightAnalog(rx, ry);
      
      vals[0] = (rx-128)/128.0;
      vals[1] = (ry-128)/128.0;
      vals[2] = -(lx-128)/128.0;
  
    } else { 
      USB<<="psx read error";
    }
  }
  for(int i=0; i<N_AXES; i++) {
    if(vals[i]!=0) move=true;
  }

  if(move && playMode) {
    USB<<="Aborting playback";
    playMode=false;
  }
  
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
        USB.println("TX:[cancel-jog]");
        stopped = true;
        lastStatusReq = millis()+200;
        requestStatus = true;
        //canSend = true;
      }
    }
  } else {
    if(canSend && rgcode<wgcode) {
      USB<<"Replaying \""<<gcodes[rgcode]<<='"';
      sendGCode(gcodes[rgcode]);
      rgcode++;
      if(rgcode==wgcode) { 
        if(!playCycle) {playMode=false;} else {sendGCode("G4 P0");rgcode=0;} 
      }
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

/*
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
 */
  
}

void sendGCode(String c) {
  GRBL<<c<<'\n';
  USB<<"TX:"<<c<<'\n';
  lastSent = millis();
  canSend = false; // wait for confirmation
}

bool addGCode(String c) {
  if(wgcode>=MAX_GCODES) {
    USB<<"Limit reached\n"; 
    return false;
  }
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
            USB<<"Parsed WCO:"<<ofsX<<" "<<ofsY<<" "<<ofsZ<<'\n';
        }

        pch = strtok(nullptr, "|"); 

    }
    
    if(!mpos) {
        x -= ofsX; y -= ofsY; z -= ofsZ;
    }

}
