#include <grl/experiments/approx_test.h>

using namespace grl;

REGISTER_CONFIGURABLE(ApproxTestExperiment)

void ApproxTestExperiment::request(ConfigurationRequest *config)
{
}

void ApproxTestExperiment::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  mapping_ = (Mapping*)config["mapping"].ptr();
  
  config.get("train_samples", train_samples_, (size_t)1000);
  config.get("test_samples", test_samples_, (size_t)1000);
  config.get("file", file_);
  
  config.get("min", min_, VectorConstructor(0.));
  config.get("max", max_, VectorConstructor(1.));
  
  grl_assert(min_.size() == max_.size());
}

void ApproxTestExperiment::reconfigure(const Configuration &config)
{
}

ApproxTestExperiment *ApproxTestExperiment::clone() const
{
  ApproxTestExperiment *ate = new ApproxTestExperiment(*this);
  ate->projector_ = projector_->clone();
  ate->representation_ = representation_->clone();
  
  return ate;
}

void ApproxTestExperiment::run() const
{
  IdentityProjector *ip = new IdentityProjector();

  for (size_t ii=0; ii < train_samples_; ++ii)
  {
    Vector in = RandGen::getVector(min_.size()), out;
    in = min_ + in*(max_-min_);

    ProjectionPtr vp = ip->project(in);
    mapping_->read(vp, &out);
    
    ProjectionPtr p = projector_->project(in);
    representation_->write(p, out);
  }
  
  std::ostream *os = &std::cout;
  std::ofstream ofs;
  
  if (file_ != "")
  {
    ofs.open(file_.c_str());
    os = &ofs;
  }
  
  for (size_t ii=0; ii < test_samples_; ++ii)
  {
    Vector in = RandGen::getVector(min_.size()), out, app;
    in = min_ + in*(max_-min_);

    ProjectionPtr vp = ip->project(in);
    mapping_->read(vp, &out);
    
    ProjectionPtr p = projector_->project(in);
    representation_->read(p, &app);
    
    double e = sum(out-app);
    
    for (size_t jj=0; jj < in.size(); ++jj)
      (*os) << in[jj] << ", ";
    for (size_t jj=0; jj < out.size(); ++jj)
      (*os) << out[jj] << ", ";
    for (size_t jj=0; jj < app.size(); ++jj)
      (*os) << app[jj] << ", ";
    (*os) << e << std::endl;
  }
}
