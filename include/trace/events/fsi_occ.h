#undef TRACE_SYSTEM
#define TRACE_SYSTEM fsi_occ

#if !defined(_TRACE_TIMER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_OCC_H

#include <linux/tracepoint.h>
#include <linux/fsi-occ.h>

TRACE_EVENT(occ_enq_xfer,
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
	TP_printk("Client %p enqueued xfer %p", __entry->client, __entry->xfer)
);

TRACE_EVENT(occ_read_complete,
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
	TP_printk("Client %p completed read for xfer %p",
		__entry->client, __entry->xfer)
);

TRACE_EVENT(occ_write_begin,
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
	TP_printk("Client %p began write for xfer %p",
		__entry->client, __entry->xfer)
);

TRACE_EVENT(occ_worker_xfer_begin,
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
	TP_printk("OCC worker began client %p xfer %p",
		__entry->client, __entry->xfer)
);

TRACE_EVENT(occ_worker_xfer_complete,
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
	TP_printk("OCC worker completed client %p xfer %p",
		__entry->client, __entry->xfer)
);

#endif

#include <trace/define_trace.h>
