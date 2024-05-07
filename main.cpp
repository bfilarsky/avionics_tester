//
//  main.cpp
//  Avionics Tester
//
//  Created by Brian Filarsky on 1/21/18.
//  Copyright © 2018 Brian Filarsky. All rights reserved.
//

#include <string>
#include <cmath>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include "hackrf.h"

using std::string;
using std::cout;
using std::endl;
using std::cin;

const unsigned SAMPLE_RATE = 1000000;
const double VOR_MOD_IDX = 0.3;
const double MB_MOD_IDX  = 0.95;
const unsigned NAV_GAIN = 30;
const unsigned MB_GAIN = 47;
const double PI = 3.1415926535;
const double MARKER_BEACON_CARRIER = 75e6;
const double OM_FREQ = 400;
const double MM_FREQ = 1300;
const double IM_FREQ = 3000;
const double VOR_FREQ = 30;
const double VOR_REF_SUBC = 9960;
const double ID_MOD_IDX = 0.1;
const double ID_SUBC = 1020;
const double VOR_AMP = floor((pow(2,7) - 1)/(1.0 + 2 * VOR_MOD_IDX + ID_MOD_IDX));
const double MB_AMP = floor((pow(2,7) - 1)/(1.0 + MB_MOD_IDX));
const double ILS_POS_LOBE_FREQ = 90.0;
const double ILS_NEG_LOBE_FREQ = 150.0;

size_t sampleNum = 0;
unsigned radial = 0;

const double GS_FREQS_MHZ[] = {
    334.7,
    334.55,
    334.1,
    333.95,
    329.9,
    329.75,
    330.5,
    330.35,
    329.3,
    329.15,
    331.4,
    331.25,
    332.0,
    331.85,
    332.6,
    332.45,
    333.2,
    333.05,
    333.8,
    333.65,
    334.4,
    334.25,
    335.0,
    334.85,
    329.6,
    329.45,
    330.2,
    330.05,
    330.8,
    330.65,
    331.7,
    331.55,
    332.3,
    332.15,
    332.9,
    332.75,
    333.5,
    333.35,
    331.1,
    330.95
};

int om_callback(hackrf_transfer* transfer){
    for (unsigned i = 0; i < transfer->buffer_length; i += 2){
        if (sampleNum % (SAMPLE_RATE / 2) < (SAMPLE_RATE / 4))
            transfer->buffer[i] = (uint8_t) (MB_AMP * (1.0 + MB_MOD_IDX * cos(2.0 * PI * OM_FREQ * sampleNum / SAMPLE_RATE)));
        else
            transfer->buffer[i] = MB_MOD_IDX;
        ++sampleNum;
    }
    return 0;
}

int mm_callback(hackrf_transfer* transfer){
    for (unsigned i = 0; i < transfer->buffer_length; i += 2){
        int mod = sampleNum % (SAMPLE_RATE / 2);
        if (mod < (SAMPLE_RATE / 4) || (mod > (4 * SAMPLE_RATE / 12) && mod < (5 * SAMPLE_RATE / 12)))
            transfer->buffer[i] = (uint8_t) (MB_AMP * (1.0 + MB_MOD_IDX * cos(2.0 * PI * MM_FREQ * sampleNum / SAMPLE_RATE)));
        else
            transfer->buffer[i] = MB_MOD_IDX;
        ++sampleNum;
    }
    return 0;
}

int im_callback(hackrf_transfer* transfer){
    for (unsigned i = 0; i < transfer->buffer_length; i += 2){
        if (sampleNum % (SAMPLE_RATE / 6) < (SAMPLE_RATE / 12))
            transfer->buffer[i] = (uint8_t) (MB_AMP * (1.0 + MB_MOD_IDX * cos(2.0 * PI * IM_FREQ * sampleNum / SAMPLE_RATE)));
        else
            transfer->buffer[i] = MB_MOD_IDX;
        ++sampleNum;
    }
    return 0;
}

int vor_callback(hackrf_transfer* transfer){
    for (size_t i = 0; i < transfer->buffer_length; i += 2){
        double time = sampleNum++ / (double)SAMPLE_RATE;
        double ident = ID_MOD_IDX * cos(2 * PI * ID_SUBC * time);
        if (sampleNum % (SAMPLE_RATE / 5) < (SAMPLE_RATE / 10))
            ident = 0;
        double ref = VOR_MOD_IDX * cos(2.0 * PI * VOR_REF_SUBC * time + 16*sin(2.0 * PI * VOR_FREQ * time));
        double var = VOR_MOD_IDX * cos(2.0 * PI * VOR_FREQ * time - radial * PI / 180.0);
        transfer->buffer[i] = (uint8_t) (VOR_AMP * (1.0 + ident + ref + var));
        transfer->buffer[i + 1] = 0;
    }
    return 0;
}

//15.5% DDM full scale?
int localizer_callback(hackrf_transfer* transfer){
    for (size_t i = 0; i < transfer->buffer_length; i += 2){
        double time = sampleNum++ / (double)SAMPLE_RATE;
        double ident = ID_MOD_IDX * cos(2 * PI * ID_SUBC * time);
        if (sampleNum % (SAMPLE_RATE / 5) < (SAMPLE_RATE / 10))
            ident = 0;
        double posIdx  = 0.4;
        double negIdx  = 0.4;
        double posLobe = posIdx*cos(2.0 * PI * ILS_POS_LOBE_FREQ * time);
        double negLobe = negIdx*cos(2.0 * PI * ILS_NEG_LOBE_FREQ * time);
        double amp = floor((pow(2,7) - 1)/(1.0 + posIdx + negIdx + ID_MOD_IDX));
        
        transfer->buffer[i] = (uint8_t) (amp * (1.0 + ident + posLobe + negLobe));
        transfer->buffer[i + 1] = 0;
    }
    return 0;
}

int glideslope_callback(hackrf_transfer* transfer){
    for (size_t i = 0; i < transfer->buffer_length; i += 2){
        double time = sampleNum++ / (double)SAMPLE_RATE;
        double posIdx  = 0.4;
        double negIdx  = 0.4;
        double posLobe = posIdx*cos(2.0 * PI * ILS_POS_LOBE_FREQ * time);
        double negLobe = negIdx*cos(2.0 * PI * ILS_NEG_LOBE_FREQ * time);
        double amp = floor((pow(2,7) - 1)/(1.0 + posIdx + negIdx));
        
        transfer->buffer[i] = (uint8_t) (amp * (1.0 + posLobe + negLobe));
        transfer->buffer[i + 1] = 0;
    }
    return 0;
}

void vorTest(hackrf_device* device){
    while(1){
        cout << "Enter VOR frequency" << endl;
        cout << "r to return to main menu" << endl;
        string option;
        cin >> option;
        cin.ignore();
        if (option[0] == 'r')
            break;
        double freq = atof(option.c_str());
        unsigned freqInt = freq * 100.0;
        if (freqInt < 10800 || freqInt > 11795){
            cout << "Invalid Entry. VOR frequencies must be in the range 108.00 - 117.95 MHz" << endl;
            continue;
        }
        if (freqInt % 5 != 0){
            cout << "Invalid Entry. Frequencies must be in 50 KHz intervals." << endl;
            continue;
        }
        if (freq > 108.0 && freq < 112.0 && freqInt % 20 >= 10){
            cout << freqInt % 20 << endl;
            cout << "Invalid Entry. The first decimal digit of VOR frequencies in the range 108.00 - 111.95 MHz must be even." << endl;
            cout << "I.e. 108.00 and 108.05 are valid, 108.10 and 108.15 are not." << endl;
            cout << "Frequncies in this range with an odd first decimal digit are reserved for localizers." << endl;
            continue;
        }
        hackrf_set_freq(device, freq*1e6);
        hackrf_start_tx(device, vor_callback, NULL);
        radial = 360;
        while(1){
            cout << "Transmitting VOR Test Signal on " << std::setprecision(2) << std::fixed << freq << " with radial set to " << std::setw(3) << std::setfill('0') << radial << endl;
            cout << "Enter radial to change, r to stop and return to VOR menu" << endl;
            cin >> option;
            cin.ignore();
            if (option[0] == 'r')
                break;
            if (atoi(option.c_str()) < 0 || atoi(option.c_str()) > 360){
                cout << "Invalid Entry. Radial must be in range 0-360" << endl;
                continue;
            }
            radial = atof(option.c_str());
        }
        hackrf_stop_tx(device);
    }
}

void localizerTest(hackrf_device* device){
    while(1){
        cout << "Enter Localizer frequency" << endl;
        cout << "r to return to main menu" << endl;
        string option;
        cin >> option;
        cin.ignore();
        if (option[0] == 'r')
            break;
        double freq = atof(option.c_str());
        unsigned freqInt = freq * 100.0;
        if (freqInt < 10810 || freqInt > 11195){
            cout << freqInt << endl;
            cout << "Invalid Entry. Localizer frequencies must be in the range 108.10 - 111.95 MHz" << endl;
            continue;
        }
        if (freqInt % 5 != 0){
            cout << "Invalid Entry. Frequencies must be in 50 KHz intervals." << endl;
            continue;
        }
        if (freqInt % 20 < 10){
            cout << freqInt % 20 << endl;
            cout << "Invalid Entry. The first decimal digit of Localizer frequencies must be odd." << endl;
            cout << "I.e. 108.10 and 108.15 are valid, 108.00 and 108.05 are not." << endl;
            cout << "Frequncies in this range with an even first decimal digit are reserved for VORs." << endl;
            continue;
        }
        freq = freqInt / 100.0;
        hackrf_set_freq(device, freq*1e6);
        hackrf_start_tx(device, localizer_callback, NULL);
        
        cout << "Transmitting Localizer Test Signal on " << std::setprecision(2) << std::fixed << freq << endl;
    }
    hackrf_stop_tx(device);
}

void glideslopeTest(hackrf_device* device){
    while(1){
        cout << "Enter Localizer frequency" << endl;
        cout << "r to return to main menu" << endl;
        string option;
        cin >> option;
        cin.ignore();
        if (option[0] == 'r')
            break;
        double freq = atof(option.c_str());
        unsigned freqInt = freq * 100.0;
        if (freqInt < 10810 || freqInt > 11195){
            cout << freqInt << endl;
            cout << "Invalid Entry. Localizer frequencies must be in the range 108.10 - 111.95 MHz" << endl;
            continue;
        }
        if (freqInt % 5 != 0){
            cout << "Invalid Entry. Frequencies must be in 50 KHz intervals." << endl;
            continue;
        }
        if (freqInt % 20 < 10){
            cout << freqInt % 20 << endl;
            cout << "Invalid Entry. The first decimal digit of Localizer frequencies must be odd." << endl;
            cout << "I.e. 108.10 and 108.15 are valid, 108.00 and 108.05 are not." << endl;
            cout << "Frequncies in this range with an even first decimal digit are reserved for VORs." << endl;
            continue;
        }
        unsigned gsIdx = (freqInt - 10810)/10;
        if (freqInt % 10)
            ++gsIdx;
        freq = freqInt / 100.0;
        double glideSlopeFreq = GS_FREQS_MHZ[gsIdx];
        hackrf_set_freq(device, glideSlopeFreq*1e6);
        hackrf_start_tx(device, glideslope_callback, NULL);
        cout << std::setprecision(2) << std::fixed << "Transmitting Glideslope Test Signal (" << glideSlopeFreq << " MHz) paired with Localizer Frequency "  << freq << " MHz" << endl;
    }
    hackrf_stop_tx(device);
}

int main() {
    hackrf_init();
    hackrf_device_list_t *dlPtr = hackrf_device_list();
    hackrf_device *device = NULL;
    while (hackrf_device_list_open(dlPtr, 0, &device) != HACKRF_SUCCESS){
        hackrf_init();
        dlPtr = hackrf_device_list();
        cout << "HackRF One not detected. Plug in the radio, then press enter" << endl;
        cin.ignore();
    }
    hackrf_device_list_free(dlPtr);
    hackrf_set_sample_rate(device, SAMPLE_RATE);
    
    while (1){
        cout << "Select Function:" << endl;
        cout << "1) VOR Test" << endl;
        cout << "2) Localizer" << endl;
        cout << "3) Glideslope" << endl;
        cout << "4) Outer Marker" << endl;
        cout << "5) Middle Marker" << endl;
        cout << "6) Inner Marker" << endl;
        cout << "q to quit" << endl;
        char option = 0;
        cin >> option;
        cin.ignore();
        if (option == 'q')
            break;
        if (option < '1' || option > '6'){
            cout << "Invalid Entry" << endl;
            continue;
        }
        switch (option) {
            case '1':
                hackrf_set_txvga_gain(device, NAV_GAIN);
                vorTest(device);
                break;
            case '2':
                hackrf_set_txvga_gain(device, NAV_GAIN);
                localizerTest(device);
                break;
            case '3':
                hackrf_set_txvga_gain(device, NAV_GAIN);
                glideslopeTest(device);
                break;
            case '4':
                hackrf_set_freq(device, MARKER_BEACON_CARRIER);
                hackrf_set_txvga_gain(device, MB_GAIN);
                hackrf_start_tx(device, om_callback, NULL);
                cout << "Transmitting Outer Marker Test Signal. Press 'enter' to stop." << endl;
                cin.ignore();
                hackrf_stop_tx(device);
                break;
            case '5':
                hackrf_set_freq(device, MARKER_BEACON_CARRIER);
                hackrf_set_txvga_gain(device, MB_GAIN);
                hackrf_start_tx(device, mm_callback, NULL);
                cout << "Transmitting Middle Marker Test Signal. Press 'enter' to stop." << endl;
                cin.ignore();
                hackrf_stop_tx(device);
                break;
            case '6':
                hackrf_set_freq(device, MARKER_BEACON_CARRIER);
                hackrf_set_txvga_gain(device, MB_GAIN);
                hackrf_start_tx(device, im_callback, NULL);
                cout << "Transmitting Inner Marker Test Signal. Press 'enter' to stop." << endl;
                cin.ignore();
                hackrf_stop_tx(device);
                break;
            default:
                break;
        }
    }
    hackrf_close(device);
    return 0;
}
