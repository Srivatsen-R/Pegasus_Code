#ifndef __STRUCTURES_H
#define __STRUCTURES_H


#include "main.h"
#include "macros.h"
#include "enums.h"
#include <stdbool.h>

struct voltage_t {
       uint8_t under,
               over;
};

struct rotor_t {
        float rpm,
              angle,
              speed,
              speed_filtered; 
};

struct phase_t {
        int   A_current,
              B_current,
              C_current; 
};

struct clarke_t {
       float alpha_current,
             beta_current;
};

struct park_t {
       float ids,
             iqs;
};

struct rho_t {
       float sin,
             cos;
};

struct vector_t {
       float err,
             gain,
             intg,
             ref,
             sen,
             u,
             excess,
             intgp,
             errp;
};

struct speed_t {
       float slip,
             sync,
             angular;
};

typedef struct terminal_t {
              float  rho_;
              struct vector_t w;
              struct vector_t id; 
              struct vector_t iq;
              struct vector_t vd;
              struct vector_t vq; 
              struct vector_t imr;      
              struct rho_t    rho;
              struct 
              rotor_t  rotor;
              struct phase_t  phase; 
              struct speed_t  speed;
              struct clarke_t clarke;
              struct park_t   park;
}terminal_t;

typedef struct __attribute__((__packed__))  temperature_t {
       int    overtemp_verifier,
              rst_temp_verifier;
       float  Rp,
              R1,
              T2,
              Bx,
              Ka;
       float  u,
              v,
              w,
              motor;
       double (*read)(double ntc);
       TEMP_STATUS (*isOverTemp)(void);

}temperature_t;

typedef struct {
       bool forward;
       bool reverse;
       bool neutral;
}fnr_state_t;

typedef struct {
       __IO uint16_t speed;
       void (*toggle) (uint16_t color);
       void (*write)  (uint16_t color);
}led_t;

typedef struct drive_t {
        char  check,
              status,
              fnr_status;

}drive_t;

typedef struct encoder_t {
        __IO int value,
                 state,
                 errorValue,
                 errorCount,
                 timeoutCount,
                 previous;
        void (*check) (void);
}encoder_t;

#endif