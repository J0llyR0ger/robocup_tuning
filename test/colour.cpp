#include "lib/colour.hpp"
using colour::classify;
using colour::Value;
static_assert(classify(0, 0, 0, 0) == Value::Black);
static_assert(classify(50, 50, 50, 150) == Value::Black);
static_assert(classify(100, 150, 400, 800) == Value::Blue);
static_assert(classify(100, 400, 150, 800) == Value::Green);
static_assert(classify(400, 100, 100, 800) == Value::Unknown);
static_assert(classify(300, 300, 300, 1000) == Value::Unknown);
static_assert(classify(100, 100, 22000, 22000) == Value::Unknown);
