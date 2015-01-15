#include <string>
#include <map>

#define DECLARE_FACTORY(x)                            \
class x ## Factory                                    \
{                                                     \
  private:                                            \
    typedef std::map<std::string, x ## Factory*> Map; \
    static Map &factories();                          \
                                                      \
  protected:                                          \
    x ## Factory(std::string name);                   \
    virtual x *create() = 0;                          \
                                                      \
  public:                                             \
    static x *create(std::string name);               \
};

#define DEFINE_FACTORY(x)                             \
x ## Factory::Map &x ## Factory::factories()          \
{                                                     \
  static Map factories_;                              \
  return factories_;                                  \
}                                                     \
                                                      \
x ## Factory::x ## Factory(std::string name)          \
{                                                     \
  std::cout << "registering " << name << std::endl;   \
  factories()[name] = this;                           \
}                                                     \
                                                      \
x* x ## Factory::create(std::string name)             \
{                                                     \
  if (!factories().count(name))                       \
    return NULL;                                      \
                                                      \
  x *obj = factories()[name]->create();               \
                                                      \
  return obj;                                         \
}

#define REGISTER_FACTORY(x, subx, name)               \
static class subx ## Factory : public x ## Factory    \
{                                                     \
  public:                                             \
    subx ## Factory() : x ## Factory(name) { }        \
    virtual x *create()                               \
    {                                                 \
      return new subx();                              \
    }                                                 \
} subx ## _factory;
