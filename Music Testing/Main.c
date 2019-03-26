//
//  Main.c
//  Botball 2018-2019
//
//  Created by RZJHS Robotics.
//  Copyright B) 2019 RZJHS Robotics. All rights reserved.
//
#include <kipr/botball.h>
#include <math.h>
typedef enum { false, true } bool;
double pos[] = {0,0,0};

int gc_song_array[16][33] = {
    //Array sequence: [Song Length] [Note Number 1] [Note Duration 1] [Note Number 2] [Note Duration 2], etc. 
    {11, 62, 8, 62, 8, 74, 16, 69, 24, 68, 8, 0, 8, 67, 16, 65, 16, 62, 8, 65, 8, 67, 8},
};

void birthday() {
    create_connect();
    create_load_song(0);
    while(get_create_song_playing(.01)) {msleep(50);}
}

int main() {
    birthday();
    return 0;
}