/*
 * common_code.h
 *
 *  Created on: Sep 17, 2015
 *      Author: manuel kudruss <manuel.kudruss@iwr.uni-heidelberg.de>
 */

#ifndef COMMON_CODE_H_
#define COMMON_CODE_H_

#include "wrapper.hpp" // MUSCOD-II interface

namespace grl {

// Thread-safe data structure
struct MuscodData {
  unsigned long NMSN, NXD, NXA, NU, NP, NH;
  bool is_initialized;
  bool got_dimensions;
  bool quit;

/*  std::string model_path;
  std::string model_name;
  std::string relative_dat_path;
*/
  std::vector<double> initial_sd; // only first shooting node contains initial values
  std::vector<double> initial_pf; // global parameters
  std::vector<std::vector<double> > qc; // all controls have to saved

  // place holder for initial solution
  std::vector<double> backup_h, backup_p;
  std::vector<std::vector<double> > backup_xd, backup_qc;

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
    backup_xd.resize(NMSN);
    backup_qc.resize(NMSN);

    muscod->getHF(backup_h.data());
    muscod->getPF(backup_p.data());

    std::vector<double> mssqp_xd(NXD);
    std::vector<double> mssqp_qc(NU);
    for (unsigned int imsn = 0; imsn < NMSN; ++imsn)
    {
      muscod->getNodeSD(imsn, mssqp_xd.data());
      muscod->getNodeQC(imsn, mssqp_qc.data());
      backup_xd[imsn] = mssqp_xd;
      backup_qc[imsn] = mssqp_qc;
    }
  };

  inline void restore_muscod_state(MUSCOD* muscod)
  {
      muscod->setHF(backup_h.data());
      muscod->setPF(backup_p.data());
      for (unsigned int imsn = 0; imsn < NMSN; ++imsn)
      {
        muscod->setNodeSD(imsn, backup_xd[imsn].data());
        muscod->setNodeQC(imsn, backup_qc[imsn].data());
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
