#ifndef STRETCH_NET_FUN
#define STRETCH_NET_FUN

#include "test.h"
#include <math.h>

#define sigmoid(x) (1.0 / (1.0 + exp(-1.0 * x)))

#define NET_INPUT 200
#define NET_HIDDEN 10

// init lenght of stretch network output in angles
#define INIT_ANGLE 129.094166667
// 2*pi*18.5
#define U 116.238928183

#define OUT_TO_MM(a) \
       (  150 + (INIT_ANGLE-a)*U/360.0)

class StretchNet {
  public:
    float net_input_ring[NET_INPUT];
    float *net_append;
    float *net_input;
    
    float net_output;
    float INPUT_SCALE;
    
    float current_value;
    
  StretchNet() {
    net_append = net_input_ring+NET_INPUT-1;
    net_input = net_input_ring;
    net_output = 0.0;
  }
    
  void add_value(float a) {
    if((net_input-net_input_ring) == NET_INPUT-1)
      net_input = net_input_ring;
    else
      net_input += 1;
      
    if((net_append-net_input_ring) == NET_INPUT-1)
      net_append = net_input_ring;
    else
       net_append += 1;
    
    *net_append = a;
    current_value = a;
  }
  
  float getNetOut() {
    float hidden[NET_HIDDEN];
    float out;
    // hidden activation
    for(int h = 0; h<NET_HIDDEN; h++) {
      hidden[h] = b_mlp[h];
      float *my_net_input = net_input;
      for(int i = 0; i<NET_INPUT; i++) {
        hidden[h] += (*my_net_input)*W_mlp[i*NET_HIDDEN+h];
      
        if( (my_net_input-net_input_ring) == NET_INPUT-1)
           my_net_input = net_input_ring;
        else
            my_net_input += 1;
      }
      hidden[h] = sigmoid(hidden[h]);
    }
  
    // out activation
    out = b_mlp1[0];
   for(int h = 0; h<NET_HIDDEN; h++) {
      out += hidden[h]*W_mlp1[h];
    }
  
    net_output = sigmoid(out);
    return net_output;
  }
};

#endif STRETCH_NET_FUN





