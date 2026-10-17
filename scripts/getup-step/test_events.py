import copy,unittest
from events import MeasuredEvents

def samples(start,n=20,**change):
    result=[]
    for i in range(start+1,start+n+1):
        s=dict(native_substep=i,loads_n=[0.,400.],clearance_m=[.025,0.],positions=[dict(x=.06,y=.025,z=0.),dict(x=0.,y=0.,z=.2)],torso_up=1.,com_height_m=.8,right_com_margin_m=.005,right_capture_margin_m=.005,combined_com_margin_m=.005,combined_capture_margin_m=.005,nonfoot_support=False,standing_without_foot_anchor=True);s.update(change);result.append(s)
    return result
class EventTests(unittest.TestCase):
    def state(self):return MeasuredEvents(0,400.,[1.,0.,0.],[0.,0.,0.])
    def observe(self,h,s,done=True,**kw):return h.observe(s,phase_reference_complete=done,actual_loaded_slip_m=[.001,.001],**kw)
    def test_time_cannot_replace_unload_and_restore_keeps_counter(self):
        h=self.state()
        for _ in range(3):self.observe(h,samples(h.native_id))
        h=MeasuredEvents(**h.state());self.assertEqual(h.phase,'prepare')
        r=self.observe(h,samples(h.native_id));self.assertEqual(r['events'],['unloaded']);self.assertEqual(h.phase,'lift')
        h=self.state();r=self.observe(h,samples(0,80,loads_n=[30.,370.]));self.assertEqual(r['events'],[])
    def test_actual_touchdown_then_separate_load_acquisition(self):
        h=self.state();h.phase='lower';h.awarded=['unloaded','clearance','forward']
        r=self.observe(h,samples(0,80,loads_n=[3.5,396.5]));self.assertEqual(h.phase,'acquire');self.assertEqual(r['events'],['touchdown'])
        r=self.observe(h,samples(80,80,loads_n=[15.,385.]));self.assertEqual(h.phase,'acquire')
        r=self.observe(h,samples(160,80,loads_n=[48.,352.]));self.assertEqual(h.phase,'settle');self.assertEqual(r['events'],['loadAcquired'])
    def test_final4000_consecutive_through_actual_end_not_longest(self):
        h=self.state();h.phase='hold';h.hold_anchors=[[.06,.025,0.],[0.,0.,.2]];h.awarded=['unloaded','clearance','forward','touchdown','loadAcquired']
        self.observe(h,samples(0,3999,loads_n=[200.,200.]));self.assertEqual(h.quiet_run,3999)
        self.observe(h,samples(3999,1,torso_up=.99,loads_n=[200.,200.]));self.assertEqual(h.quiet_run,0)
        r=self.observe(h,samples(4000,4000,loads_n=[200.,200.]));self.assertEqual(r['events'],['quietComplete'])
        self.assertEqual(self.observe(h,samples(8000,20,loads_n=[200.,200.]))['events'],[])
    def test_missing_support_hull_and_corrupt_ids_reject_before_mutation(self):
        for bad in ['right_capture_margin_m','native_substep']:
            h=self.state();s=samples(0);s[4].pop(bad)
            with self.assertRaises(ValueError):self.observe(h,s)
            self.assertEqual(h.native_id,0)
    def test_clearance_failure_and_slip_do_not_grant_progress(self):
        h=self.state();h.phase='advance';h.awarded=['unloaded','clearance'];r=self.observe(h,samples(0,20,clearance_m=[.0149,0.]));self.assertEqual(r['physicalFailure'],'advance_clearance');self.assertEqual(r['events'],[])
if __name__=='__main__':unittest.main()
