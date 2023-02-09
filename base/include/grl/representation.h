/** \file representation.h
 * \brief Generic representation definitions.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
 */

#ifndef GRL_REPRESENTATION_H_
#define GRL_REPRESENTATION_H_

#include <pthread.h>

#include <grl/configurable.h>
#include <grl/projection.h>
#include <grl/trace.h>

namespace grl
{

/// Approximates a Mapping.
class Representation : public Configurable
{
  private:
    std::vector<ProjectionPtr> batch_;
    pthread_mutex_t mutex_;

  public:
    Representation() : mutex_(PTHREAD_MUTEX_INITIALIZER) { }
  
    /// Read out the approximation.
    virtual double read(const ProjectionPtr &projection, Vector *result) const
    {
      return read(projection, result, NULL);
    }

    /// Read out the approximation, returning standard deviation of result.
    virtual double read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const = 0;

    /// Add a new estimate of the target function, using a single learning rate for all outputs.
    virtual void write(const ProjectionPtr projection, const Vector &target, double alpha=1.)
    {
      Vector valpha;
      valpha = ConstantVector(target.size(), alpha);
      write(projection, target, valpha);
    }
    
    /// Add a new estimate of the target function, using separate learning rates per output.
    virtual void write(const ProjectionPtr projection, const Vector &target, const Vector &alpha) = 0;
    
    /// Adjust an estimate of the target function.
    virtual void update(const ProjectionPtr projection, const Vector &delta)
    {
      Vector value;
      read(projection, &value);
      write(projection, value+delta);
    }
    
    /// Update a trace of target function estimates
    virtual void update(const Trace &trace, const Vector &delta, double e=1.)
    {
      for (Trace::iterator ii=trace.begin(); ii != trace.end() && ii->weight() > 0.001; ++ii)
        update(ii->projection(), ii->weight()*delta*e);
    }
    
    /// Apply written values, if not already done on-line.
    virtual void finalize() { }
    
    /// Returns the Jacobian of the representation around the input projection.
    virtual Matrix jacobian(const ProjectionPtr projection) const
    {
      return Matrix();
    }

    /// Starts a batch read operation.
    virtual void batchRead(size_t sz)
    {
      // Make sure other threads can't interrupt our batch
      pthread_mutex_lock(&mutex_);
    
      batch_.clear();
      batch_.reserve(sz);
    }

    /// Starts a batch write operation.
    virtual void batchWrite(size_t sz)
    {
    }
    
    /// Enqueues a batch read operation, to be read later.
    virtual void enqueue(const ProjectionPtr &projection)
    {
      batch_.push_back(projection);
    }

    /// Enqueues a batch write operation, to be executed later.
    virtual void enqueue(const ProjectionPtr &projection, const LargeVector &target)
    {
      // Assume write is not reentrant for regular representations
      write(projection, target);
    }
    
    /// Reads the results of all enqueued read operations.
    virtual void read(Matrix *out)
    {
      Vector result;
      read(batch_[0], &result);
      if (!result.size())
      {
        *out = Matrix();
        return;
      }
      
      *out = Matrix(batch_.size(), result.size());
      out->row(0) = result;
      
      #ifdef _OPENMP
      #pragma omp parallel for
      #endif
      for (size_t ii=1; ii < batch_.size(); ++ii)
      {
        Vector result;
        read(batch_[ii], &result);
        out->row(ii) = result;
      }

      pthread_mutex_unlock(&mutex_);
    }
    
    /// Executes all enqueued write operations.
    virtual void write()
    {
      finalize();
    }
    
    /// Returns the representation to use when calculating targets.
    virtual Representation *target()
    {
      return this;
    }
};

/// Representation that allows for parameter access.
class ParameterizedRepresentation : public Representation
{
  protected:
    int interval_;
    double tau_;
    std::string file_suffix_;
    int count_;
    ParameterizedRepresentation *target_;
    pthread_mutex_t mutex_;

  public:
    ParameterizedRepresentation() : interval_(0), tau_(1), count_(0), target_(NULL), mutex_(PTHREAD_MUTEX_INITIALIZER) { }

    // From Configurable
    void request(const std::string &role, ConfigurationRequest *config)
    {
      config->push_back(CRP("interval", "Target representation update interval (number of writes; 0=never update)", interval_, CRP::Configuration));
      config->push_back(CRP("tau", "Target representation update strength", tau_, CRP::Configuration));
      config->push_back(CRP("file_suffix", "Suffix for loading and saving parameters (default: configuration path)", file_suffix_, CRP::Configuration));

      config->push_back(CRP("target", d_type() + "." + role, "Target representation", CRP::Provided));
    }
    
    void configure(Configuration &config)
    {
      interval_ = config["interval"];
      tau_ = config["tau"];
      file_suffix_ = config["file_suffix"].str();
      
      if (interval_)
      {
        target_ = (ParameterizedRepresentation*) reinstantiate({"interval"});
        config.set("target", target_);
      }
      else
        config.set("target", this);
    }
    
    void reconfigure(const Configuration &config)
    {
      std::string file;
    
      if (config.has("file"))
        file = config["file"].str();
        
      if (file_suffix_.empty())
      {
        std::string cfg_path = path();
        std::replace(cfg_path.begin(), cfg_path.end(), '/', '_');
        file += cfg_path + ".dat";
      }
      else
        file += file_suffix_ + ".dat";
    
      if (config.has("action") && config["action"].str() == "save")
      {
        FILE *f = fopen(file.c_str(), "wb");
        if (!f)
        {
          WARNING("Could not open '" << file << "' for writing");
          return;
        }
        
        LargeVector p = params(); 
        
        fwrite(p.data(), sizeof(double), p.size(), f);
        fclose(f);
      }
      
      if (config.has("action") && config["action"].str() == "load")
      {
        LargeVector p = params(); 
      
        FILE *f = fopen(file.c_str(), "rb");
        if (!f)
        {
          WARNING("Could not open '" << file << "' for reading");
          return;
        }
        
        fseek(f, 0, SEEK_END);
        if (ftell(f) != (long int)(p.size() * sizeof(double)))
        {
          WARNING("Configuration mismatch for '" << file << "'");
          fclose(f);
          return;
        }
        
        fseek(f, 0, SEEK_SET);
        if (fread(p.data(), sizeof(double), p.size(), f) != p.size())
        {
          WARNING("Could not read '" << file << "'");
          fclose(f);
          return;
        }
        fclose(f);
        
        setParams(p);
        synchronize();
      }
    }

    // From Representation
    virtual ParameterizedRepresentation *target()
    {
      if (!interval_)
        return this;
      else
        return target_;
    }

    /// Returns number of parameters.
    virtual size_t size() const = 0;
    
    /// Returns constant parameter vector.
    virtual const LargeVector &params() const = 0;
    
    /// Sets parameter vector.
    virtual void setParams(const LargeVector &params) = 0;
    
  protected:
    void synchronize(bool force=true)
    {
      if (!interval_)
        return;
    
      TRACE("Synchronizing target representation with strength " << tau_);
      
      if (tau_)
        target_->setParams(tau_*params() + (1-tau_)*target_->params());
      else
        target_->setParams(params());
        
      count_ = 0;
    }
    
    void checkSynchronize()
    {
      pthread_mutex_lock(&mutex_);
      count_++;
      if (interval_ && count_ >= interval_)
        synchronize(false);
      pthread_mutex_unlock(&mutex_);
    }
};

}

#endif /* GRL_REPRESENTATION_H_ */
