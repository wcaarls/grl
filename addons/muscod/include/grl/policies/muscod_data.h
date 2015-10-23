/*
 * common_code.h
 *
 *  Created on: Sep 17, 2015
 *      Author: manuel kudruss <manuel.kudruss@iwr.uni-heidelberg.de>
 */

#ifndef COMMON_CODE_H_
#define COMMON_CODE_H_

#include <grl/vector.h>
#include "wrapper.hpp" // MUSCOD-II interface

namespace grl {

// Thread-safe data structure
struct MuscodData {
  unsigned long NMSN, NXD, NXA, NU, NP, NH;
  bool is_initialized;
  bool got_dimensions;
  bool quit;

  Vector sd; // only first shooting node contains initial values
  Vector pf; // global parameters
  Matrix qc; // all controls have to saved

  // place holder for initial solution
  Vector backup_h, backup_p;
  Matrix backup_xd, backup_qc;

  // TODO add thread-safe getter and setter for initial values and controls
  inline void get_muscod_dimensions(MUSCOD* muscod)
  {
      NMSN = muscod->getNMSN(0);
      muscod->getNH(&NH);
      muscod->getNP(&NP);
      muscod->getDimIMSN(0, &NXD, &NXA, &NU);
  };

  inline void backup_muscod_state(MUSCOD* muscod)
  {
    if (!got_dimensions)
    {
      get_muscod_dimensions(muscod);
    }

    backup_h.resize(NH);
    backup_p.resize(NP);
    backup_xd = Matrix::Constant(NMSN, NXD, 0.0);
    backup_qc = Matrix::Constant(NMSN, NU,  0.0);

    muscod->getHF(backup_h.data());
    muscod->getPF(backup_p.data());

    Vector mssqp_xd = ConstantVector(NXD, 0.0);
    Vector mssqp_qc = ConstantVector(NU,  0.0);
    for (unsigned int imsn = 0; imsn < NMSN; ++imsn)
    {
      muscod->getNodeSD(imsn, mssqp_xd.data());
      muscod->getNodeQC(imsn, mssqp_qc.data());
      backup_xd.row(imsn) = mssqp_xd;
      backup_qc.row(imsn) = mssqp_qc;
    }
  };

  inline void restore_muscod_state(MUSCOD* muscod)
  {
      muscod->setHF(backup_h.data());
      muscod->setPF(backup_p.data());
      for (unsigned int imsn = 0; imsn < NMSN; ++imsn)
      {
        muscod->setNodeSD(imsn, backup_xd.row(imsn).data());
        muscod->setNodeQC(imsn, backup_qc.row(imsn).data());
      }
  };

  // Constructor
  MuscodData() :
    NMSN(0),
    NXD(0),
    NXA(0),
    NU(0),
    NP(0),
    quit(false),
    is_initialized(false),
    got_dimensions(false)
  {}
};

} /* namespace grl */

#endif /* COMMON_CODE_H_ */
