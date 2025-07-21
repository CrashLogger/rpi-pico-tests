
#include "pico/stdlib.h"
#include <stdio.h>
#include "string.h"
#include "stdio.h"
#include "GPSPARSE.h"

class GPSPARSE{

    //VARS
        float latitude = 0;
        float longitude = 0;
    
    //FUNCS

        GPSPARSE::~GPSPARSE(){
        }


        uint8_t GPSPARSE::parse_location(char* sentence){
            
            char prefix[6];
            strncpy(prefix, sentence + 1,5);
            //printf("%s\n",prefix);
            
            //strcmp returns 0 when both are equal!
            if(!strcmp(prefix, "GNRMC")){
                printf("%s", sentence);
                char sentencePart[128];
                memcpy(sentencePart,&sentence[7],strlen(sentence)-6);
                
                char buffer[128];
                uint8_t field_counter = 0;

                for(int i = 0; i<strlen(sentence); i++){

                    if(sentence[i]==','){

                        if(field_counter==3){
                            latitude = lat_clean(buffer, sentence[i+1]);
                        }
                        else if (field_counter==5){
                            longitude = long_clean(buffer, sentence[i+1]);
                        }

                        printf("\n");
                        field_counter++;
                        //Clearing the buffer between sentence sections
                        memset(buffer, 0, sizeof buffer);
                        printf("%d\t",field_counter);
                    }
                    else{
                        buffer[strlen(buffer)] = sentence[i];
                    }
                }

            }
            else if(!strcmp(prefix, "GPTXT")){
            //printf("Diagnostics sentence!");
            }
            return(0);
        }




float lat_clean(char* raw_numeric, char direction){
    printf("%s, %c", raw_numeric, direction);

    // Dealing with the degrees in the GPS sentence
    char loc_deg[3];
    sprintf(loc_deg, "%c%c", raw_numeric[0], raw_numeric[1]);
    float loc_coo = 0.0f;
    loc_coo = loc_coo + atoi(loc_deg);

    // Dealing with the decimal minutes in the GPS sentence
    raw_numeric[strlen(raw_numeric)] = 0;
    printf(&raw_numeric[2]);
    loc_coo = loc_coo + (atof(&raw_numeric[2]))/60.0f;

    //N and E are both positive and before the letter O
    //S and W are both negative and after the letter O
    if(direction > 'O'){
        loc_coo = loc_coo - 2*loc_coo;;
    }

    printf("\tPARSED LOCAL COORDINATE:%4.4f", loc_coo);

    return(loc_coo);
}

float long_clean(char* raw_numeric, char direction){
    printf("%s, %c", raw_numeric, direction);

    // Dealing with the degrees in the GPS sentence
    char loc_deg[4];
    sprintf(loc_deg, "%c%c%c", raw_numeric[0], raw_numeric[1], raw_numeric[2]);
    float loc_coo = 0.0f;
    loc_coo = loc_coo + atoi(loc_deg);

    // Dealing with the decimal minutes in the GPS sentence
    raw_numeric[strlen(raw_numeric)] = 0;
    printf(&raw_numeric[3]);
    loc_coo = loc_coo + (atof(&raw_numeric[3]))/60.0f;

    

    //N and E are both positive and before the letter O
    //S and W are both negative and after the letter O
    if(direction > 'O'){
        loc_coo = loc_coo - 2*loc_coo;
    }

    printf("\tPARSED LOCAL COORDINATE:%4.4f", loc_coo);

    return(loc_coo);
}

}

