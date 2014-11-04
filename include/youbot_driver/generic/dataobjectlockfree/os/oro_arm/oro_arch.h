/***************************************************************************
 * File copied from
 * https://github.com/psoetens/orocos-rtt/blob/arm-port/src/os/oro_atomic.h
  
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU General Public                   *
 *   License as published by the Free Software Foundation;                 *
 *   version 2 of the License.                                             *
 *                                                                         *
 *   As a special exception, you may use this file as part of a free       *
 *   software library without restriction.  Specifically, if other files   *
 *   instantiate templates or use macros or inline functions from this     *
 *   file, or you compile this file and link it with other files to        *
 *   produce an executable, this file does not by itself cause the         *
 *   resulting executable to be covered by the GNU General Public          *
 *   License.  This exception does not however invalidate any other        *
 *   reasons why the executable file might be covered by the GNU General   *
 *   Public License.                                                       *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public             *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef __ARCH_arm_ORO_ATOMIC__
#define __ARCH_arm_ORO_ATOMIC__

#define oro_localirq_save_hw_notrace(x)					\
	({							\
		unsigned long temp;				\
		(void) (&temp == &x);				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ oro_localirq_save_hw\n"	\
"	orr	%1, %0, #128\n"					\
"	msr	cpsr_c, %1"					\
	: "=r" (x), "=r" (temp)					\
	:							\
	: "memory", "cc");					\
	})
#define oro_localirq_save_hw(flags)	oro_localirq_save_hw_notrace(flags)

#define oro_localirq_restore_hw_notrace(x)				\
	__asm__ __volatile__(					\
	"msr	cpsr_c, %0		@ oro_localirq_restore_hw\n"	\
	:							\
	: "r" (x)						\
	: "memory", "cc")
#define oro_localirq_restore_hw(flags)	oro_localirq_restore_hw_notrace(flags)	

typedef struct { volatile int counter; } oro_atomic_t;

#define ORO_ATOMIC_INIT(i)	{ (i) }

#define oro_atomic_read(v)	((v)->counter)

#define oro_atomic_set(v,i)	(((v)->counter) = (i))

static inline int oro_atomic_add_return(int i, oro_atomic_t *v)
{
	unsigned long flags;
	int val;

	oro_localirq_save_hw(flags);
	val = v->counter;
	v->counter = val += i;
	oro_localirq_restore_hw(flags);

	return val;
}

static inline int oro_atomic_sub_return(int i, oro_atomic_t *v)
{
	unsigned long flags;
	int val;

	oro_localirq_save_hw(flags);
	val = v->counter;
	v->counter = val -= i;
	oro_localirq_restore_hw(flags);

	return val;
}

static inline int oro_atomic_cmpxchg(oro_atomic_t *v, int old, int a)
{
	int ret;
	unsigned long flags;

	oro_localirq_save_hw(flags);
	ret = v->counter;
		v->counter = a;
	oro_localirq_restore_hw(flags);

	return ret;
}

static inline void oro_atomic_clear_mask(unsigned long mask, unsigned long *addr)
{
	unsigned long flags;

	oro_localirq_save_hw(flags);
	*addr &= ~mask;
	oro_localirq_restore_hw(flags);
}

#define oro_atomic_xchg(v, new) (xchg(&((v)->counter), new))

static inline int oro_atomic_add_unless(oro_atomic_t *v, int a, int u)
{
	int c, old;

	c = oro_atomic_read(v);
	while (c != u && (old = oro_atomic_cmpxchg((v), c, c + a)) != c)
		c = old;
	return c != u;
}
#define oro_atomic_inc_not_zero(v) oro_atomic_add_unless((v), 1, 0)

#define oro_atomic_add(i, v)	(void) oro_atomic_add_return(i, v)
#define oro_atomic_inc(v)		(void) oro_atomic_add_return(1, v)
#define oro_atomic_sub(i, v)	(void) oro_atomic_sub_return(i, v)
#define oro_atomic_dec(v)		(void) oro_atomic_sub_return(1, v)

#define oro_atomic_inc_and_test(v)	(oro_atomic_add_return(1, v) == 0)
#define oro_atomic_dec_and_test(v)	(oro_atomic_sub_return(1, v) == 0)
#define oro_atomic_inc_return(v)    (oro_atomic_add_return(1, v))
#define oro_atomic_dec_return(v)    (oro_atomic_sub_return(1, v))
#define oro_atomic_sub_and_test(i, v) (oro_atomic_sub_return(i, v) == 0)

#define oro_atomic_add_negative(i,v) (oro_atomic_add_return(i, v) < 0)

/* oro_atomic operations are already serializing on ARM */
#define smp_mb__before_oro_atomic_dec()	barrier()
#define smp_mb__after_oro_atomic_dec()	barrier()
#define smp_mb__before_oro_atomic_inc()	barrier()
#define smp_mb__after_oro_atomic_inc()	barrier()

#endif // __ARCH_arm_ORO_atomic__
