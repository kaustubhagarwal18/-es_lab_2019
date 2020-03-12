//
// Created by koen on 9-5-19.
//

#include "state_switch.h"
#include <pthread.h> // needed for starting threads
#include <unistd.h> // needed for usleep
#include <stdio.h>
#include <stdlib.h>


void *update_mode() {
    char buffer[4];
    char *input;
    int inp_num;

    while (1) {
        printf("Enter state 0 (SAFE) 1 (PANIC) 2 (MANUAL): ");
        input = fgets(buffer, 4, stdin);
        inp_num = (int) strtol(input, NULL, 10);
        if (inp_num > 2 || inp_num < -1) {
            fprintf(stderr, "Error: input number range exceeded.\n");
            continue;
        } else if (inp_num == -1)
            return 0;
        current_mode = inp_num;
        fflush(stdout);
    }
}

char *get_mode_name(mode m) {
    switch (m) {
        case SAFE:
            return "SAFE";
        case PANIC:
            return "PANIC";
        case MANUAL:
            return "MANUAL";
    }
}

int blah(int i){
    i += 1;
    return i;
}

int main(void) {
//    current_mode = SAFE;
//
//    mode last_mode = current_mode;
//
//    printf("\n----DRONE----\nState: %s\n-------------\n", get_mode_name(current_mode));
//
//    pthread_t receiver;
//    if (pthread_create(&receiver, NULL, update_mode, NULL)) {
//        printf("Could not start receiver thread\n");
//        return -1;
//    }
//
//
//    while (1) {
//        if (last_mode != current_mode) {
//            printf("\n----DRONE----\nState: %s\n-------------\n", get_mode_name(current_mode));
//            last_mode = current_mode;
//        }
//
//        switch (current_mode) {
//            case SAFE:
//                break;
//            case PANIC:
//                break;
//            case MANUAL:
//                break;
//        }
//
//
//
//        usleep(1000000);
//    }

    printf("%d\n", blah(1));

    return 0;
}