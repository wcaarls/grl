#include <grl/projectors/tile_coding.h>

using namespace grl;

REGISTER_CONFIGURABLE(TileCodingProjector)

void TileCodingProjector::request(ConfigurationRequest *config)
{
}

void TileCodingProjector::configure(const Configuration &config)
{
  Vector res = config["resolution"];

  config.get("tilings", tilings_, 16);
  config.get("memory", memory_, 8);
  memory_ *= 1024*1024;
  
  config.get("wrapping", wrapping_, Vector());

  if (wrapping_.empty())
    wrapping_.resize(res.size(), 0.);
    
  if (wrapping_.size() != res.size())
    throw bad_param("projector/tile_coding:wrapping");
  
  for (size_t ii=0; ii < res.size(); ++ii)
  {
    scaling_.push_back(tilings_/res[ii]);
    wrapping_[ii] *= scaling_[ii];
    if (fabs(wrapping_[ii]-round(wrapping_[ii])) > 0.001)
      throw bad_param("projector/tile_coding:wrapping");
    wrapping_[ii] = round(wrapping_[ii]);
  }
}

void TileCodingProjector::reconfigure(const Configuration &config)
{
}

TileCodingProjector *TileCodingProjector::clone() const
{
  TileCodingProjector *tc = new TileCodingProjector();
  tc->tilings_ = tilings_;
  tc->memory_ = memory_;
  tc->scaling_ = scaling_;
  tc->wrapping_ = wrapping_;
  
  return tc;
}

#define MAX_NUM_VARS 32

ProjectionPtr TileCodingProjector::project(const Vector &in) const
{
  int num_floats = in.size();
  int i,j;
  int qstate[MAX_NUM_VARS];
  int base[MAX_NUM_VARS];
  int coordinates[MAX_NUM_VARS + 1];   /* one interval number per relevant dimension */
  int num_coordinates = num_floats + 1;
  
  grl_assert(num_floats == scaling_.size());
  
  IndexProjection *p = new IndexProjection();
  p->indices.resize(tilings_);

  /* quantize state to integers (henceforth, tile widths == numTilings) */
  for (i = 0; i < num_floats; i++)
  {
    qstate[i] = (int) floor(in[i] * scaling_[i]);
    base[i] = 0;
  }

  /*compute the tile numbers */
  for (j = 0; j < tilings_; j++)
  {
    /* loop over each relevant dimension */
    for (i = 0; i < num_floats; i++)
    {
      /* find coordinates of activated tile in tiling space */
      coordinates[i] = qstate[i] - safe_mod((qstate[i] - base[i]), tilings_);

      if (wrapping_[i] != 0)
        coordinates[i] = safe_mod(coordinates[i], (int)wrapping_[i]);

      /* compute displacement of next tiling in quantized space */
      base[i] += 1 + (2 * i);
    }
    /* add additional indices for tiling and hashing_set so they hash differently */
    coordinates[i] = j;
    p->indices[j] = getFeatureLocation(coordinates, num_coordinates);
  }
  
  return ProjectionPtr(p);
}
