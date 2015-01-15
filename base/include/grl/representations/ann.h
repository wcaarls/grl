/*
 * ann_representation.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef ANN_REPRESENTATION_H_
#define ANN_REPRESENTATION_H_

#include <grl/representation.h>

namespace grl
{

/// Artificial neural network.
class ANNRepresentation : public ParameterizedRepresentation
{
  public:
    TYPEINFO("representation/parameterized/ann")
};

}

#endif /* ANN_REPRESENTATION_H_ */
