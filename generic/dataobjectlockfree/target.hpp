
# if defined( __GNUC__ ) && defined( __i386__ )
#  define OROBLD_OS_ARCH_i386
# elif defined( __GNUC__ ) && defined( __x86_64__ )
#  define OROBLD_OS_ARCH_x86_64
# elif defined( __GNUC__ ) && (defined( __powerpc__ ) || defined( __PPC__ ) )
#  define OROBLD_OS_ARCH_ppc
# elif defined( __GNUC__ ) && (defined( __arm__ ) || defined( __ARM__ ) )
#  define OROBLD_OS_ARCH_arm
# elif defined( __GNUC__ ) && defined( __ia64__ )
#  error "ia64 Is not yet supported, contact the orocos-dev mailinglist for further actions."
#  define OROBLD_OS_ARCH_ia64
# elif defined( __MINGW32__ ) && defined( __i386__ )
#  define OROBLD_OS_ARCH_i386
# elif defined( WIN32 )
#  define OROBLD_OS_ARCH_i386
# else
#  error "Unknown Processor Architecture"
#  define OROBLD_OS_ARCH_unknown
# endif
