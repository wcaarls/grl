/** \file stub.cpp
 * \brief Tensorflow representation stub.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-04-07
 *
 * \copyright \verbatim
 * Copyright (c) 2018, Wouter Caarls
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

#include <dlfcn.h>

#include <grl/configurable.h>
#include <grl/representations/tensorflow_api.h>

using namespace grl;

TF_DEFINE(Version);

TF_DEFINE(NewStatus);
TF_DEFINE(DeleteStatus);
TF_DEFINE(GetCode);
TF_DEFINE(Message);;

TF_DEFINE(NewBufferFromString);
TF_DEFINE(DeleteBuffer);

TF_DEFINE(AllocateTensor);
TF_DEFINE(DeleteTensor);
TF_DEFINE(NumDims);
TF_DEFINE(Dim);
TF_DEFINE(TensorData);

TF_DEFINE(NewSessionOptions);
TF_DEFINE(DeleteSessionOptions);

TF_DEFINE(NewGraph);
TF_DEFINE(DeleteGraph);
TF_DEFINE(GraphOperationByName);
TF_DEFINE(GraphGetTensorShape);

TF_DEFINE(NewImportGraphDefOptions);
TF_DEFINE(DeleteImportGraphDefOptions);
TF_DEFINE(GraphImportGraphDef);

TF_DEFINE(NewSession);
TF_DEFINE(DeleteSession);
TF_DEFINE(SessionRun);

static class TFInit
{
  public:
    TFInit()
    {
      NOTICE("Loading TensorFlow library");

      // The whole reason we're doing the stub thing is so we can load libtensorflow.so
      // with RTLD_DEEPBIND, so that it doesn't load the symbols of the protobuf
      // version our addon is linked with.
      void *handle = dlopen("libtensorflow.so", RTLD_NOW|RTLD_LOCAL|RTLD_DEEPBIND);
      if (!handle)
        ERROR("Error loading tensorflow library 'libtensorflow.so': " << dlerror());
        
      // Resolve symbols
      TF_RESOLVE(Version);

      TF_RESOLVE(NewStatus);
      TF_RESOLVE(DeleteStatus);
      TF_RESOLVE(GetCode);
      TF_RESOLVE(Message);

      TF_RESOLVE(NewBufferFromString);
      TF_RESOLVE(DeleteBuffer);

      TF_RESOLVE(AllocateTensor);
      TF_RESOLVE(DeleteTensor);
      TF_RESOLVE(NumDims);
      TF_RESOLVE(Dim);
      TF_RESOLVE(TensorData);

      TF_RESOLVE(NewSessionOptions);
      TF_RESOLVE(DeleteSessionOptions);

      TF_RESOLVE(NewGraph);
      TF_RESOLVE(DeleteGraph);
      TF_RESOLVE(GraphOperationByName);
      TF_RESOLVE(GraphGetTensorShape);

      TF_RESOLVE(NewImportGraphDefOptions);
      TF_RESOLVE(DeleteImportGraphDefOptions);
      TF_RESOLVE(GraphImportGraphDef);

      TF_RESOLVE(NewSession);
      TF_RESOLVE(DeleteSession);
      TF_RESOLVE(SessionRun);
    }
} tfinit__;
