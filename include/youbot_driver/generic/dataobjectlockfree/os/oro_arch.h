//#include "../rtt-config.h"


#  if defined(_MSC_VER)
#   include "youbot_driver/generic/dataobjectlockfree/os/oro_msvc/oro_arch.h"
#  elif defined(__GNUC__)
#    if ( OROBLD_GCC_VERSION < 40100 || defined(__clang__) )
#      if defined( OROBLD_OS_ARCH_i386 )
#         include "youbot_driver/generic/dataobjectlockfree/os/oro_i386/oro_arch.h"
#      elif defined( OROBLD_OS_ARCH_x86_64 )
#         include "youbot_driver/generic/dataobjectlockfree/os/oro_x86_64/oro_arch.h"
#      elif defined( OROBLD_OS_ARCH_ppc )
#         include "youbot_driver/generic/dataobjectlockfree/os/oro_powerpc/oro_arch.h"
#      elif defined( OROBLD_OS_ARCH_arm )
#         include "youbot_driver/generic/dataobjectlockfree/os/oro_arm/oro_arch.h"
#      else
#         error "Unsupported architecture for GCC < 4.1.0 or clang++"
#      endif
#    else // GCC >= 4.1.0 has intrinsics. Clang++'s 2.9 intrinsics are broken in C++ code
#      include "youbot_driver/generic/dataobjectlockfree/os/oro_gcc/oro_arch.h"
#    endif
#  else
#   error "Unsupported architecture or compiler"
#  endif
