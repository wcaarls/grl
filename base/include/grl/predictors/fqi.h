/*
 * fqi_predictor.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef FQI_PREDICTOR_H_
#define FQI_PREDICTOR_H_

#include <grl/predictor.h>

namespace grl
{

/// Fitted Q-iteration predictor.
class FQIPredictor : public BatchPredictor
{
  public:
    TYPEINFO("predictor/fqi")
};

}

#endif /* FQI_PREDICTOR_H_ */
