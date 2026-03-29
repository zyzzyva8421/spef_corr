import time
import spef_rc_correlation as m

m.HAS_CPP = True
s1 = m.SpefFile('20-blabla.spef')
s2 = m.SpefFile('20-blabla_new_shuffled.spef')

t0 = time.perf_counter()
s1.parse()
s2.parse()
print('parse', time.perf_counter() - t0)

t1 = time.perf_counter()
caps, ress, tcap, tres = m.compare_spef(s1, s2, 'max')
print('cmp', time.perf_counter() - t1)
print('lens', len(caps), len(ress), len(tcap), len(tres))
