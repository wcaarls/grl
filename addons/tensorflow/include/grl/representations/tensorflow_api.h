#include <memory>
#include <iostream>

#include <grl/vector.h>

#define TF_DECLARE(x) extern TF_##x x
#define TF_DEFINE(x) TF_##x TF::x
#define TF_RESOLVE(x) TF::x = (TF_##x) dlsym(handle, "TF_" #x)

// TensorFlow C API types
extern "C"
{
  typedef enum TF_DataType { TF_FLOAT = 1, TF_BOOL = 10 } TF_DataType;
  typedef enum TF_Code { TF_OK = 0 } TF_Code;

  typedef struct TF_Status TF_Status;
  typedef struct TF_Buffer TF_Buffer;
  typedef struct TF_Tensor TF_Tensor;
  typedef struct TF_ImportGraphDefOptions TF_ImportGraphDefOptions;
  typedef struct TF_Graph TF_Graph;
  typedef struct TF_Operation TF_Operation;
  typedef struct TF_SessionOptions TF_SessionOptions;
  typedef struct TF_Session TF_Session;
  typedef struct TF_Output
  {
    TF_Operation* oper;
    int index;
  } TF_Output;

  typedef const char* (*TF_Version)();

  typedef TF_Status* (*TF_NewStatus)();
  typedef void (*TF_DeleteStatus)(TF_Status*);
  typedef TF_Code (*TF_GetCode)(const TF_Status*);
  typedef const char* (*TF_Message)(const TF_Status*);

  typedef TF_Buffer* (*TF_NewBufferFromString)(const void*, size_t);
  typedef void (*TF_DeleteBuffer)(TF_Buffer*);

  typedef TF_Tensor* (*TF_AllocateTensor)(TF_DataType, const int64_t*, int, size_t);
  typedef void (*TF_DeleteTensor)(TF_Tensor*);
  typedef int (*TF_NumDims)(const TF_Tensor*);
  typedef int64_t (*TF_Dim)(const TF_Tensor*, int);
  typedef void* (*TF_TensorData)(const TF_Tensor*);

  typedef TF_SessionOptions* (*TF_NewSessionOptions)();
  typedef void (*TF_SetConfig)(TF_SessionOptions*, const void*, size_t, TF_Status*);
  typedef void (*TF_DeleteSessionOptions)(TF_SessionOptions*);

  typedef TF_Graph* (*TF_NewGraph)();
  typedef void (*TF_DeleteGraph)(TF_Graph*);
  typedef TF_Operation* (*TF_GraphOperationByName)(TF_Graph*, const char*);
  typedef void (*TF_GraphGetTensorShape)(TF_Graph*, TF_Output, int64_t*, int, TF_Status*);

  typedef TF_ImportGraphDefOptions* (*TF_NewImportGraphDefOptions)();
  typedef void (*TF_DeleteImportGraphDefOptions)(TF_ImportGraphDefOptions*);
  typedef void (*TF_GraphImportGraphDef)(TF_Graph*, const TF_Buffer*, const TF_ImportGraphDefOptions*, TF_Status*);

  typedef TF_Session* (*TF_NewSession)(TF_Graph*, const TF_SessionOptions*, TF_Status*);
  typedef void (*TF_DeleteSession)(TF_Session*, TF_Status*);
  typedef void (*TF_SessionRun)(TF_Session*, const TF_Buffer*,
    const TF_Output*, TF_Tensor* const*, int,
    const TF_Output*, TF_Tensor**, int,
    const TF_Operation* const*, int,
    TF_Buffer*, TF_Status*);
}

namespace TF
{
  TF_DECLARE(Version);

  TF_DECLARE(NewStatus);
  TF_DECLARE(DeleteStatus);
  TF_DECLARE(GetCode);
  TF_DECLARE(Message);

  TF_DECLARE(NewBufferFromString);
  TF_DECLARE(DeleteBuffer);

  TF_DECLARE(AllocateTensor);
  TF_DECLARE(DeleteTensor);
  TF_DECLARE(NumDims);
  TF_DECLARE(Dim);
  TF_DECLARE(TensorData);

  TF_DECLARE(NewSessionOptions);
  TF_DECLARE(SetConfig);
  TF_DECLARE(DeleteSessionOptions);

  TF_DECLARE(NewGraph);
  TF_DECLARE(DeleteGraph);
  TF_DECLARE(GraphOperationByName);
  TF_DECLARE(GraphGetTensorShape);

  TF_DECLARE(NewImportGraphDefOptions);
  TF_DECLARE(DeleteImportGraphDefOptions);
  TF_DECLARE(GraphImportGraphDef);

  TF_DECLARE(NewSession);
  TF_DECLARE(DeleteSession);
  TF_DECLARE(SessionRun);
  
  class Shape
  {
    protected:
      std::vector<int64_t> dims_;
      
    public:
      Shape() { }
      Shape(std::initializer_list<int64_t> dims) : dims_(dims) { }
      Shape(std::vector<int64_t> dims) : dims_(dims) { }
      Shape(const grl::Vector &v) : dims_(v.size()) { }
      Shape(const grl::Matrix &m) : dims_ {m.rows(), m.cols()} { }
      Shape(TF_Tensor* tensor);
      
      const int64_t *dims() const { return dims_.data(); }
      int num_dims() const { return dims_.size(); }
      
      size_t size() const
      {
        size_t sz=1;
        for (size_t ii=0; ii != dims_.size(); ++ii)
          sz *= dims_[ii];
          
        return sz;
      }
      
      friend std::ostream& operator<<(std::ostream& os, const Shape& shape)
      {
        os << shape.dims_;
        return os;
      }
  };

  class Tensor
  {
    protected:
      TF_Tensor *tensor_;
      float *data_;
      size_t stride_;
      
    public:
      Tensor() : tensor_(NULL), data_(NULL), stride_(0) { }
      Tensor(TF_Tensor* tensor);
      Tensor(const Shape &shape);
      Tensor(const grl::Vector &v, Shape shape=Shape());
      Tensor(const grl::Matrix &m, Shape shape=Shape());
      
      ~Tensor();
      
      float operator ()(size_t i) const { return data_[i]; }
      float &operator ()(size_t i) { return data_[i]; }
      float operator ()(size_t r, size_t c) const { return data_[r*stride_+c]; }
      float &operator ()(size_t r, size_t c) { return data_[r*stride_+c]; }
      operator TF_Tensor*() { return tensor_; }
      operator grl::Vector();
      operator grl::Matrix();
      
      Shape shape() { return Shape(tensor_); }
      float *data();
      
    private:
      Tensor &operator=(const Tensor&) { return *this; }
  };

  typedef struct std::shared_ptr<Tensor> TensorPtr;
}
