#include <libgen.h>

#include <QtGui>
#include <grl/environments/odesim/dialog.h>
#include <grl/environments/odesim/environment.h>

using namespace grl;

ODEDialog::ODEDialog(ODESTGEnvironment *env)
{
  setupUi(this); // this sets up GUI

  // Set sim
  mGLWidget->setSim(env->getSim()->getSim());
  mGLWidget->init();  // Do not forget to initialize the GLWidget
  setWindowFlags(Qt::WindowMaximizeButtonHint);

  mGLWidget->dsSetShadows(0);

  // signals/slots mechanism
  connect(env, SIGNAL(drawFrame()), this, SLOT(drawGLWidget()));
  connect(mCbxEnableGraphics, SIGNAL(toggled(bool)), this, SLOT(enableGraphics(bool)));
  
  mGLWidget->update();
  show();
}

void ODEDialog::drawGLWidget()
{
  mGLWidget->update();
}

void ODEDialog::enableGraphics(bool enabled)
{
  mGLWidget->enableGraphics(enabled);
  mGLWidget->update();
}
