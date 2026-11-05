#include <cl_camera_plugins/state.hpp>
#include <cassert>
#include <iostream>
using namespace cl_camera_plugins;
int main() {
  ExpiringMarks m;
  m.add(0.4,0,0,0.05); m.add(0,0.4,1,0.05);
  assert(m.snapshot(1.99,2).size()==2);
  assert(m.snapshot(2,2).size()==1); // earlier viewing direction expires
  m.add(0,0.4,2.5,0.05);
  assert(m.snapshot(3.1,2).size()==1); // continued observation refreshes TTL
  assert(m.snapshot(4.5,2).empty());
  m.add(-0.02,-0.02,5,0.05); m.add(0.02,0.02,5,0.05);
  assert(m.snapshot(5.1,2).size()==2); // quantization handles negative coordinates
  m.clear(); assert(m.snapshot(5.2,2).empty());
  FreeCounter f;
  for(int i=0;i<4;++i) f.observe(false,i*0.1,1);
  assert(!f.ready(0.35,1,5));
  f.observe(false,0.4,1); assert(f.ready(0.5,1,5));
  assert(!f.ready(1.5,1,5)); // stale free data cannot end recovery
  f.observe(false,2,1); assert(!f.ready(2,1,5)); // gap resets count
  f.observe(true,2.1,1); assert(!f.ready(2.1,1,1));
  std::cout << "Expiry, refresh, coordinate and free-message tests passed.\n";
}
