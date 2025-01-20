#include "kalman.h"
#include "TimersAndCalculations.h"

float Kalman::updateEstimate( float mea, kalman_ts * filter )
{
    filter->kalman_gain = filter->err_estimate / (filter->err_estimate + filter->err_measure);
    filter->current_estimate = filter->last_estimate + filter->kalman_gain * (mea - filter->last_estimate);
    filter->err_estimate = (1.0f - filter->kalman_gain) * filter->err_estimate + fsc_fabs(filter->last_estimate - filter->current_estimate) * filter->q;
    filter->last_estimate = filter->current_estimate;

    return filter->current_estimate;
}
