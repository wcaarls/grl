#ifndef ODESIM_DIALOG_H_
#define ODESIM_DIALOG_H_

#include <QtGui/QDialog>

#include "ui_odesim_dialog.h"

namespace grl
{

class ODEDialog : public QMainWindow, public Ui::ODEDialog
{
  Q_OBJECT
  
  protected:
    class ODESTGEnvironment *env_;

  public:
    ODEDialog(class ODESTGEnvironment *env);

  public slots:
    void enableGraphics(bool);
    void drawGLWidget();
};

}

#endif /* ODESIM_DIALOG_H_ */
