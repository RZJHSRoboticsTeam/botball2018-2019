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
    //Serial sequence: [140] [Song Number] [Song Length] [Note Number 1] [Note Duration 1] [Note Number 2] [Note Duration 2], etc. 
    {16, 294}
}

void birthday() {
    create_connect();
    int gc_song_array[16][33];
    gc_song_array[0][0]=16;
    gc_song_array[0][1]=0;
    gc_song_array[0][2]=1;
    gc_song_array[0][3]=32;
    gc_song_array[0][4]=1;
    gc_song_array[0][5]=0;
    gc_song_array[0][6]=1;
    gc_song_array[0][7]=60;
    gc_song_array[0][8]=22;
    gc_song_array[0][9]=60;
    gc_song_array[0][10]=7;
    gc_song_array[0][11]=62;
    gc_song_array[0][12]=29;
    gc_song_array[0][13]=60;
    gc_song_array[0][14]=29;
    gc_song_array[0][15]=53;
    gc_song_array[0][16]=26;
    create_load_song(0);
    while(get_create_song_playing(.01)) {msleep(50);}
}

int main() {
    birthday();
    return 0;
}