#include <iostream>
#include <gl_wrapper/object/pipe.hpp>

using namespace gl_wrapper;

Pipe::Pipe(double h, double r_in, double r_out, unsigned int slices, unsigned int stacks)
  : h_(h), r_in_(r_in), r_out_(r_out), slices_(slices), stacks_(stacks)
{
  model_list_ = glGenLists(1);
}

void Pipe::displayImpl()
{
  glutSolidCylinder(r_in_, h_, slices_, stacks_);
}
