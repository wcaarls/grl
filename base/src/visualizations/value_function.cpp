#include <GL/gl.h>
#include <GL/glu.h>

#include <grl/visualizations/value_function.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(ValueFunctionVisualization) 

void ValueFunctionVisualization::request(ConfigurationRequest *config)
{
}

void ValueFunctionVisualization::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (QPolicy*)config["policy"].ptr();

  state_min_ = config["min"];
  state_max_ = config["max"];
  state_dims_ = state_min_.size();
  config.get("points", points_, 1048576);

  // Create point iteration order lookup table  
  Vector dims = VectorConstructor(0., 1.);
  config.get("dims", dims);
  dim_order_.clear();
  for (int ii=0; ii < state_dims_; ++ii)
    if (ii != dims[0] && ii != dims[1])
      dim_order_.push_back(ii);
  dim_order_.push_back(dims[0]);
  dim_order_.push_back(dims[1]);
  
  // Divide points among dimensions
  dimpoints_ = pow(points_, 1./state_dims_);
  points_ = pow(dimpoints_, state_dims_);
  texpoints_ = dimpoints_*dimpoints_;
  
  // Allocate texture
  data_ = (unsigned char*) malloc(texpoints_*3*sizeof(unsigned char));
  
  // Create window  
  create("Value function");
}

void ValueFunctionVisualization::reconfigure(const Configuration &config)
{
}

void ValueFunctionVisualization::reshape(int width, int height)
{
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, 0, 1, 1);
  glMatrixMode(GL_MODELVIEW);
}

// TODO: split off into new thread
void ValueFunctionVisualization::idle()
{
  float *value = new float[texpoints_];

  const Vector delta = (state_max_-state_min_)/(dimpoints_-1);

  // Gather data
  Vector ss = state_min_;
  float value_max=-std::numeric_limits<float>::infinity(),
        value_min= std::numeric_limits<float>::infinity();
  
  for (int ii=0; ii < texpoints_; ++ii)
  {
    float v = 0;//-std::numeric_limits<float>::infinity();
  
    for (int jj=0; jj < points_/texpoints_; ++jj)
    {
      Vector action, q;
      policy_->act(ss, &action);
      representation_->read(projector_->project(ss, action), &q);
      v += q[0];
      
      for (int dd=0; dd < state_dims_; ++dd)
      {
        int oo = dim_order_[dd];
    
        ss[oo] = ss[oo] + delta[oo];
        if (ss[oo] > (state_max_[oo]+EPS))
          ss[oo] = state_min_[oo];
        else
          break;
      }
    }

    v /= points_/texpoints_;

    value[ii] = v;
    value_max = fmax(v, value_max);
    value_min = fmin(v, value_min);
  }
  
  float value_range = value_max-value_min;
  
  // Create texture
  for (int ii=0; ii < texpoints_; ++ii)
  {
    double v = (value[ii] - value_min)/value_range;
    double v2 = 4*v;
    
    // Jet colormap
    data_[ii*3+0] = fmax(fmin(255*fmin(v2 - 1.5, -v2 + 4.5), 255), 0);
    data_[ii*3+1] = fmax(fmin(255*fmin(v2 - 0.5, -v2 + 3.5), 255), 0);
    data_[ii*3+2] = fmax(fmin(255*fmin(v2 + 0.5, -v2 + 2.5), 255), 0);
  }
  
  delete[] value;
  
  value_min_ = value_min;
  value_max_ = value_max;

  // Redisplay  
  updated_ = true;
  refresh();
}

void ValueFunctionVisualization::draw()
{
  if (updated_)
  {
    if (!texture_)
      glGenTextures(1, &texture_);
  
    // Set texture
    glBindTexture( GL_TEXTURE_2D, texture_ );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    
    gluBuild2DMipmaps( GL_TEXTURE_2D, 3, dimpoints_, dimpoints_, GL_RGB, GL_UNSIGNED_BYTE, data_ );
    
    updated_ = false;
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  // Draw texture
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_);

  glBegin(GL_QUADS);
    glTexCoord2d(0.0,0.0); glVertex2d(-1.0,-1.0);
    glTexCoord2d(1.0,0.0); glVertex2d(1.0,-1.0);
    glTexCoord2d(1.0,1.0); glVertex2d(1.0,1.0);
    glTexCoord2d(0.0,1.0); glVertex2d(-1.0,1.0);
  glEnd();
  
  glDisable(GL_TEXTURE_2D);
/*  
  char buf[255];
  sprintf(buf, "%8.2f - %8.2f", value_min_, value_max_);
  
  glColor3f(1.0, 1.0, 1.0);
  glRasterPos2f(-1.0, -1.0);
  glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)buf);
  
  glBegin(GL_LINES);
  glVertex2d(state_[0]-0.05, state_[1]);
  glVertex2d(state_[0]+0.05, state_[1]);
  glVertex2d(state_[0], state_[1]-0.05);
  glVertex2d(state_[0], state_[1]+0.05);
  glEnd();
*/ 
  swap();
}
