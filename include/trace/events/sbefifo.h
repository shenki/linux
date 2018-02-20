#undef TRACE_SYSTEM
#define TRACE_SYSTEM sbefifo

#if !defined(_TRACE_TIMER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SBEFIFO_H

#include <linux/tracepoint.h>
#include <linux/fsi-sbefifo.h>

TRACE_EVENT(sbefifo_new_client,
	TP_PROTO(const void *client),
	TP_ARGS(client),
	TP_STRUCT__entry(
		__field(const void *, client)
	),
	TP_fast_assign(
		__entry->client = client;
	),
	TP_printk("New client: %p", __entry->client)
);

TRACE_EVENT(sbefifo_release_client,
	TP_PROTO(const void *client),
	TP_ARGS(client),
	TP_STRUCT__entry(
		__field(const void *, client)
	),
	TP_fast_assign(
		__entry->client = client;
	),
	TP_printk("Released client: %p", __entry->client)
);

TRACE_EVENT(sbefifo_enq_xfer,
	TP_PROTO(const void *client, const void *xfer),
	TP_ARGS(client, xfer),
	TP_STRUCT__entry(
		__field(const void *, client)
		__field(const void *, xfer)
	),
	TP_fast_assign(
		__entry->client = client;
		__entry->xfer = xfer;
	),
	TP_printk("Client %p enqueued transfer %p",
		  __entry->client, __entry->xfer)
);

TRACE_EVENT(sbefifo_begin_xfer,
	TP_PROTO(const void *xfer),
	TP_ARGS(xfer),
	TP_STRUCT__entry(
		__field(const void *, xfer)
	),
	TP_fast_assign(
		__entry->xfer = xfer;
	),
	TP_printk("Began transfer %p",
		  __entry->xfer)
);

TRACE_EVENT(sbefifo_end_xfer,
	TP_PROTO(const void *xfer, int ret),
	TP_ARGS(xfer, ret),
	TP_STRUCT__entry(
		__field(const void *, xfer)
		__field(int, ret)
	),
	TP_fast_assign(
		__entry->xfer = xfer;
		__entry->ret = ret;
	),
	TP_printk("Completed transfer %p: %d",
		  __entry->xfer, __entry->ret)
);

TRACE_EVENT(sbefifo_deq_xfer,
	TP_PROTO(const void *client, const void *xfer),
	TP_ARGS(client, xfer),
	TP_STRUCT__entry(
		__field(const void *, client)
		__field(const void *, xfer)
	),
	TP_fast_assign(
		__entry->client = client;
		__entry->xfer = xfer;
	),
	TP_printk("Client %p dequeueing transfer %p",
		  __entry->client, __entry->xfer)
);
#endif

#include <trace/define_trace.h>
