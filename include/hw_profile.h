#pragma once
#if defined(HW_PROFILE_V1)
  #include "hw_profile_v1.h"
#elif defined(HW_PROFILE_V2)
  #include "hw_profile_v2.h"
#else
  #error "No hardware profile selected (set -DHW_PROFILE_V1 or V2)"
#endif