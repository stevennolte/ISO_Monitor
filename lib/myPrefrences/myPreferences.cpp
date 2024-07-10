#include "Arduino.h"
#include "Preferences.h"

class myPrefrences{
    private:
    
    public:
        
        myPrefrences(){}
        Preferences prefs;

        void setup()
        {
            prefs.begin("setup",false);
        }
        
        uint8_t * getIPAddr()
        {
            uint8_t ips[4];
            ips[0] = prefs.getInt("IP1",192);
            ips[1] = prefs.getInt("IP2",168);
            ips[2] = prefs.getInt("IP3",1);
            ips[3] = prefs.getInt("IP4",250);
            return ips;
        }

        bool setIPAddr(uint8_t ip4)
        {
            if (prefs.putInt("IP4",ip4)==0){
                return false;
            }

            return true;
        }
        bool setNetworkIPAddr(uint8_t ips[3]){
            prefs.putInt("IP1", ips[0]);
            prefs.putInt("IP2", ips[1]);
            prefs.putInt("IP3", ips[2]);
            return true;
        }
};
