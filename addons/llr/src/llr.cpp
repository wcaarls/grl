#include <grl/representations/llr.h>
#include <grl/projections/neighbor.h>

using namespace grl;

REGISTER_CONFIGURABLE(LLRRepresentation)

void LLRRepresentation::request(ConfigurationRequest *config)
{
}

void LLRRepresentation::configure(Configuration &config)
{
  projector_ = (NeighborProjector*)config["projector"].ptr();
}

void LLRRepresentation::reconfigure(const Configuration &config)
{
}

LLRRepresentation *LLRRepresentation::clone() const
{
  return NULL;
}

double LLRRepresentation::read(const ProjectionPtr &projection, Vector *result) const
{
  return 0;
}

void LLRRepresentation::write(const ProjectionPtr projection, const Vector &target, double alpha)
{
  // Push query on store
}

void LLRRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
}
                