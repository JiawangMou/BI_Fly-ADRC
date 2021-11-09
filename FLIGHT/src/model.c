#include "model.h"
#include "arm_math.h"
#include "tf.h"

tf_t servo_dyn_TF1;
static float servo_dyn_TF1_deno_coeff[SERVO_DYN_TF_DIMENSION];
static float servo_dyn_TF1_nume_coeff[SERVO_DYN_TF_DIMENSION];
static float servo_dyn_TF1_state_x[SERVO_DYN_TF_DIMENSION];
static float servo_dyn_TF1_state_y[SERVO_DYN_TF_DIMENSION];

tf_t servo_dyn_TF2;
static float servo_dyn_TF2_deno_coeff[SERVO_DYN_TF_DIMENSION];
static float servo_dyn_TF2_nume_coeff[SERVO_DYN_TF_DIMENSION];
static float servo_dyn_TF2_state_x[SERVO_DYN_TF_DIMENSION];
static float servo_dyn_TF2_state_y[SERVO_DYN_TF_DIMENSION];

void model_init(void)
{
    servo_dyn_TF1.deno_coeff = servo_dyn_TF1_deno_coeff;
    servo_dyn_TF1.nume_coeff = servo_dyn_TF1_nume_coeff;
    servo_dyn_TF1.order = SERVO_DYN_TF_ORDER;
    servo_dyn_TF1.state_x = servo_dyn_TF1_state_x;
    servo_dyn_TF1.state_y = servo_dyn_TF1_state_y;    

    servo_dyn_TF2.deno_coeff = servo_dyn_TF2_deno_coeff;
    servo_dyn_TF2.nume_coeff = servo_dyn_TF2_nume_coeff;
    servo_dyn_TF2.order = SERVO_DYN_TF_ORDER;
    servo_dyn_TF2.state_x = servo_dyn_TF2_state_x;
    servo_dyn_TF2.state_y = servo_dyn_TF2_state_y;    
}

float Model_servoangle_est(const u32 intput,tf_t *tf)
{
    float output = 0;
    float denAccum = (intput - -1.906 * test_DW.DiscreteTransferFcn_states[0])
    - 0.909 * test_DW.DiscreteTransferFcn_states[1];
    return output; 
}

uint32_t model_update(void)
{

}

