#ifndef NCO_CONF_H
#define NCO_CONF_H

#ifdef __cplusplus /* if C++, specify external linkage to C functions */
extern "C" {
#endif

int nco_counter_send_conf(const char *filename,
		const int freqHz_ref, const double freqHz_out,
		const int accum_size, const int offset,
		const char pinc_sw, const char poff_sw);
int nco_counter_set_max_accum(const char *filename, const uint64_t max);

#ifdef __cplusplus
}
#endif

#endif /*NCO_CONF_H*/
