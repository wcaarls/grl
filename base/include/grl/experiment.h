/*
 * experiment.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

#include <grl/configurable.h>

namespace grl
{

/// Runs an experiment.
class Experiment : public Configurable
{
  public:
    virtual ~Experiment() { }
    virtual Experiment *clone() const = 0;
    virtual void run() const = 0;
};

}

#endif /* EXPERIMENT_H_ */
