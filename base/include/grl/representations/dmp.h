/*
 * dmp_representation.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef DMP_REPRESENTATION_H_
#define DMP_REPRESENTATION_H_

#include <grl/representation.h>

namespace grl
{

/// Dynamic movement primitive.
class DMPRepresentation : public ParameterizedRepresentation
{
  public:
    TYPEINFO("representation/parameterized/dmp")
};

}

#endif /* DMP_REPRESENTATION_H_ */
