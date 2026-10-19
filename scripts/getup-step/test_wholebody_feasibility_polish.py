"""Portable mathematical tests. All minimize calls are MOCKS; no gait solve."""
import json,math,unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from coupled_posture_reference import digest,Jet
from wholebody_math_fixture import make_math_fixture
import wholebody_feasibility_polish as polish


def stage_for(knots):
    return dict(sourceRecoverySHA256='a'*64,sourceDiagnosticSHA256='b'*64,sourceCandidateMajorIteration=33,
        sourceCandidateSHA256=digest(np.asarray(knots).tolist()),sourceRecoveryOptimizerCalls=1,
        sourceRecoveryOptimizerSuccess=False,sourceRecoveryTermination='compute-cap')


class PolishTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.f=make_math_fixture();cls.knots=np.tile(np.asarray(cls.f.lower.position)[cls.f.realizer.independent_indices],(3,1))

    def evaluator(self):
        f=self.f
        return polish.AnalyticPolishEvaluator(f.realizer,f.context,f.limits,f.heading,f.support,self.knots,
            design_only=True,entry_elapsed=f.context.elapsed)

    def run_mock(self,harness,**extra):
        f=self.f
        with patch.object(polish,'minimize',side_effect=harness):
            return polish.polish_horizon(f.realizer,f.context,f.limits,f.heading,f.support,self.knots,
                stage_binding=stage_for(self.knots),design_only=True,entry_elapsed=f.context.elapsed,input_binding=f.binding,**extra)

    def test_proxy_repairs_only_evaluation_domain_and_never_extrapolates(self):
        original=self.f.arm_map;proxy=polish.OptimizerRawDomainArmMap(original)
        p=np.array([original.ranges[0][0]+.02-4e-11,0.]);j=Jet.rest(p)
        with self.assertRaisesRegex(ValueError,'verified map/reserve'):original.evaluate(j)
        self.assertEqual(proxy.evaluate(j),original.evaluate(j,reserve=0.))
        self.assertIs(proxy.original,original);self.assertEqual(proxy.ranges,original.ranges)
        for value in(original.ranges[0][0]-1e-13,original.ranges[0][1]+1e-13):
            with self.assertRaisesRegex(ValueError,'RAW'):proxy.evaluate(Jet.rest([value,0.]))

    def test_evaluator_analytic_rows_exact_cache_and_fixed_original_owner(self):
        e=self.evaluator();before=self.f.context.receipt();x=self.knots.reshape(27)
        values,rows=e.evaluate(x);again,again_rows=e.evaluate(x.copy())
        self.assertEqual(len(e.nodes),13);self.assertEqual(values.shape,(13*len(e.labels),))
        np.testing.assert_array_equal(values.view(np.uint64),again.view(np.uint64))
        self.assertIs(values,again);self.assertIs(rows,again_rows)
        with self.assertRaises(ValueError):values[0]=1.
        self.assertIs(e.r.arm_map,self.f.arm_map);self.assertIs(e.entry.realizer,e.r)
        self.assertIs(e.path(x,strict_map=True).realizer,e.r)
        self.assertIs(e.path(x).realizer.arm_map.original,e.r.arm_map)
        # Independently sample only the portable math fixture at the same node.
        path=e.path(x);sample=path.sample('lift',e.entry.durations[0]*.25)
        block=e.constraint_block(sample)
        # The unoptimized synthetic far landing can reject the full horizon;
        # compare this cached valid lift node, never relabel that rejection.
        cached_block=next(v[0]for v in e.node_cache.values()if v[1]['phase']=='lift'and v[1]['fraction']==.25)
        np.testing.assert_array_equal(cached_block.view(np.uint64),block.view(np.uint64))
        self.assertEqual(block[-3],polish.MAX_TOTAL_COM_ACCELERATION-np.linalg.norm(sample.com.acceleration))
        delta=np.asarray(sample.geometric.future_arm_delta_right_left.position)
        np.testing.assert_array_equal(block[42:44],delta-np.asarray(self.f.arm_map.ranges)[:,0]-.02)
        self.assertEqual(self.f.context.receipt(),before)
        self.assertEqual(e.stats['entryTemplates'],1);self.assertGreaterEqual(e.stats['fullCandidateHits'],1)
        self.assertTrue(e.cache_receipt()['noMutableIKSeed'])

    def test_minimum_norm_objective_original_settings_and_no_projection(self):
        class Stop(RuntimeError):pass
        seen={}
        def harness(fun,x,**kwargs):
            seen.update(kwargs);np.testing.assert_array_equal(x,self.knots.reshape(27))
            self.assertEqual(fun(x),0.);np.testing.assert_array_equal(kwargs['jac'](x),np.zeros(27))
            d=np.arange(27)*1e-7;self.assertAlmostEqual(fun(x+d),.5*float(d@d),places=20)
            np.testing.assert_allclose(kwargs['jac'](x+d),d,atol=1e-16)
            raise Stop('mock only, no real optimizer')
        # No source/native solve: mock certificate sampling to retain rejection.
        with patch.object(polish.WholeBodyCycle,'sample',side_effect=ValueError('mathematical forced rejection')):
            path,r=self.run_mock(harness)
        self.assertIsNone(path);self.assertFalse(r['optimizerSuccess']);self.assertEqual(r['optimizerCalls'],1)
        self.assertEqual(r['totalProducerOptimizerCalls'],2);self.assertEqual(r['version'],polish.VERSION_POLISH)
        self.assertEqual(seen['method'],'SLSQP');self.assertEqual(seen['options'],dict(maxiter=80,ftol=1e-12,eps=1e-6,disp=False))
        self.assertEqual(len(seen['bounds']),27);self.assertEqual(r['twoStageFeasibilityRestoration']['maximumCorrectionRad'],0.)
        self.assertFalse(r['twoStageFeasibilityRestoration']['sourceRecoveryOptimizerSuccess'])
        self.assertTrue(r['twoStageFeasibilityRestoration']['declared123SampleCertificate'])
        self.assertEqual(len(r['analyticSamples']),123);self.assertEqual(len(r['analyticFailures']),123)
        self.assertFalse(r['finalOriginalStrictMapRestored']);self.assertFalse(r['nativeAdmission'])
        self.assertEqual(r['finalCandidateOrigin'],'unchanged-bound-source-candidate')

    def test_ordered_strict_certificate_includes_all_failed_samples(self):
        def harness(fun,x,**kwargs):return SimpleNamespace(x=x,success=True,message='MOCK protocol only',nit=0)
        calls=[]
        def reject(path,phase,t,**kwargs):
            self.assertIs(path.realizer,self.f.realizer);self.assertIs(path.realizer.arm_map,self.f.arm_map)
            calls.append((phase,t));raise ValueError('mock strict map rejection')
        with patch.object(polish.WholeBodyCycle,'sample',new=reject):path,r=self.run_mock(harness)
        expected=[(p,f)for p in polish.PHASES for f in polish.CERTIFICATE_FRACTIONS]
        self.assertEqual([(x['phase'],x['fraction'])for x in r['analyticSamples']],expected)
        self.assertEqual(len(calls),123);self.assertFalse(r['finiteSampledKinematicPass']);self.assertIsNone(path)
        self.assertEqual(r['twoStageFeasibilityRestoration']['certificateReceipt']['completedSampleCount'],123)
        self.assertFalse(r['twoStageFeasibilityRestoration']['certificateReceipt']['optimizerProxyUsed'])

    def test_progress_is_detached_and_cooperative_cancel_keeps_exact_last_major(self):
        seen=[];cancel=[False];changed=self.knots.reshape(27).copy();changed[0]+=1e-7
        def progress(event):
            seen.append(json.loads(json.dumps(event)));event['iteration']=999
            event['unadmittedIndependentKnotsRad'][0][0]=999.
        def harness(fun,x,**kwargs):
            kwargs['callback'](changed);cancel[0]=True;fun(x)
            raise AssertionError('cancel must escape')
        path,r=self.run_mock(harness,progress_callback=progress,cancellation_check=lambda:cancel[0])
        self.assertIsNone(path);self.assertEqual(r['terminalState'],'caller-cancelled');self.assertFalse(r['optimizerSuccess'])
        self.assertEqual(r['completedCallbackIterations'],1);self.assertEqual(r['progressRecords'][0]['iteration'],1)
        self.assertEqual(r['finalCandidateOrigin'],'last-completed-major-iteration')
        np.testing.assert_array_equal(r['unadmittedFinalIndependentKnotsRad'],changed.reshape(3,9))
        self.assertEqual(r['progressRecords'][0]['candidateSHA256'],digest(changed.reshape(3,9).tolist()))
        self.assertFalse(seen[0]['nativeAdmission']);self.assertFalse(r['twoStageFeasibilityRestoration']['certificateReceipt']['passed'])
        self.assertEqual(r['twoStageFeasibilityRestoration']['certificateReceipt']['completedSampleCount'],0)

    def test_binding_and_controls_reject_before_any_optimizer(self):
        f=self.f
        for kwargs in(dict(max_wall_seconds=180.01),dict(certificate_max_wall_seconds=120.01),dict(max_iterations=81),dict(progress_callback=True)):
            with patch.object(polish,'minimize',side_effect=AssertionError('real optimizer forbidden')):
                with self.assertRaises(ValueError):
                    polish.polish_horizon(f.realizer,f.context,f.limits,f.heading,f.support,self.knots,
                        stage_binding=stage_for(self.knots),design_only=True,**kwargs)
        bad=stage_for(self.knots);bad['sourceCandidateMajorIteration']=32
        with patch.object(polish,'minimize',side_effect=AssertionError('real optimizer forbidden')):
            with self.assertRaisesRegex(ValueError,'stage mismatch'):
                polish.polish_horizon(f.realizer,f.context,f.limits,f.heading,f.support,self.knots,stage_binding=bad,design_only=True)

    def test_certificate_cap_is_separate_from_optimizer_cap(self):
        def harness(fun,x,**kwargs):return SimpleNamespace(x=x,success=True,message='MOCK protocol only',nit=0)
        clock=[0.]
        def advance(path,phase,t,**kwargs):clock[0]=121.;raise ValueError('mock delayed sample')
        with patch.object(polish.time,'monotonic',side_effect=lambda:clock[0]),patch.object(polish.WholeBodyCycle,'sample',new=advance):
            path,r=self.run_mock(harness)
        self.assertTrue(r['optimizerSuccess']);self.assertFalse(r['accepted']);self.assertIsNone(path)
        self.assertEqual(r['optimizerElapsedS'],0.);self.assertEqual(r['terminalState'],'certificate-compute-cap')
        self.assertLess(len(r['analyticSamples']),123)

    def test_rejected_newton_is_explicit_fixed_size_not_success(self):
        e=self.evaluator()
        with patch.object(polish.WholeBodyCycle,'sample',side_effect=ValueError('fixed Newton branch did not converge')):
            c,rows=e.evaluate(self.knots.reshape(27))
        self.assertTrue(np.all(c==-1.));self.assertEqual(len(c),13*len(e.labels))
        self.assertIn('Newton',e.exceptions[0]['message']);self.assertIn('error',rows[-1])


class RefinementTests(PolishTests):
    # Inherited portable contracts also exercise the backwards-compatible
    # unrefined profile. Every trajectory/time optimizer remains a mock.
    def refinement(self):
        nodes={p:[f for k,f in polish.normalized_constraint_nodes()if polish.PHASES[k]==p]for p in polish.PHASES}
        added=[dict(phase='lift',fraction=.8),dict(phase='translate',fraction=.625),dict(phase='land',fraction=.725)]
        for x in added:nodes[x['phase']].append(x['fraction'])
        brackets=[];results=[]
        for node in added:
            j=min(range(41),key=lambda j:abs(polish.CERTIFICATE_FRACTIONS[j]-node['fraction']))
            bracket=list(polish.CERTIFICATE_FRACTIONS[j-1:j+2]);phase=node['phase']
            brackets.append(dict(phase=phase,worstSampleFraction=bracket[1],bracketFractions=bracket))
            results.append(dict(phase=phase,success=True,fraction=node['fraction'],minimumSlack=-.01,
                timeS=node['fraction']*(.8-self.f.context.elapsed,1.2,1.)[polish.PHASES.index(phase)],evaluations=3,iterations=3))
        binding=dict(schema=1,sourceFailedPolishSHA256='c'*64,addedConstraintNodes=added,
            selectionPolicy='worst-failing-bracket-extremum-per-phase-v1',anchorUnchanged=True,onlyOneRefinement=True,
            intermediateFailedPolishOptimizerCalls=1,refinementOptimizerCalls=1,
            selectionInputCandidateSHA256='d'*64,selectionReceiptSHA256='e'*64,
            sourceBrackets=brackets,bracketExtremaResults=results)
        return nodes,binding

    def fake_certificate(self,*,dense_failure=False,scalar_success=True):
        f=self.f;durations=(.8-f.context.elapsed,1.2,1.)
        def harness(fun,x,**kwargs):
            np.testing.assert_array_equal(x,self.knots.reshape(27));self.assertEqual(fun(x),0.)
            return SimpleNamespace(x=x,success=True,message='MOCK refinement protocol only',nit=0)
        def sample(path,phase,t,**kwargs):
            self.assertIs(path.realizer,f.realizer)
            T=durations[polish.PHASES.index(phase)];p=np.array(f.context.com.position)
            p[0]+=.001*math.sin(2*math.pi*t/T)
            return SimpleNamespace(com=Jet(p,[.05+.01*math.sin(2*math.pi*t/T),0.,0.],[.2-.01*(t-.4*T)**2,0.,0.]))
        def row(path,sample,k,fraction,*args):
            accepted=not(dense_failure and k==0 and 0.<fraction<.01)
            return dict(phase=polish.PHASES[k],fraction=fraction,accepted=accepted,
                originalStrictMapEvaluationPassed=True,kneeReferenceGainDeg=1.)
        scalar_calls=[]
        def scalar(fun,*,bounds,method,options):
            scalar_calls.append((bounds,method,options));t=sum(bounds)/2.;v=fun(t)
            return SimpleNamespace(x=t,fun=v,success=scalar_success,nfev=1,nit=1)
        nodes,binding=self.refinement()
        with patch.object(polish.WholeBodyCycle,'sample',new=sample),patch.object(polish,'_sample_row',side_effect=row),\
                patch.object(polish,'summarize_samples',return_value={}),patch.object(polish,'minimize_scalar',side_effect=scalar):
            path,r=self.run_mock(harness,constraint_nodes=nodes,collocation_refinement=binding,
                max_wall_seconds=300.,certificate_max_wall_seconds=180.)
        return path,r,scalar_calls

    def test_refinement_nodes_keep_original13_and_exact_anchor(self):
        nodes,binding=self.refinement();normalized=polish.normalized_constraint_nodes(nodes)
        self.assertEqual(len(normalized),16);self.assertTrue(set(polish.normalized_constraint_nodes()).issubset(normalized))
        e=polish.AnalyticPolishEvaluator(self.f.realizer,self.f.context,self.f.limits,self.f.heading,self.f.support,self.knots,
            design_only=True,constraint_nodes=nodes)
        self.assertEqual(e.nodes,normalized);np.testing.assert_array_equal(e.anchor,self.knots.reshape(27))
        self.assertEqual(polish.refinement_binding(binding,normalized,e.anchor)['refinedConstraintNodeCount'],16)
        nodes['lift'].remove(0.)
        with self.assertRaisesRegex(ValueError,'original13'):polish.normalized_constraint_nodes(nodes)

    def test_dense_and_bracketed_extrema_profile_all_pass_without_real_optimizers(self):
        path,r,calls=self.fake_certificate();self.assertIsNotNone(path);self.assertTrue(r['accepted'])
        self.assertEqual(r['version'],polish.VERSION_REFINEMENT);self.assertEqual(r['totalProducerOptimizerCalls'],3)
        self.assertEqual(r['optimizerCalls'],1);self.assertEqual(r['trajectoryOptimizerCalls'],1)
        self.assertEqual(r['collocationRefinement']['selectionPolicy'],'worst-failing-bracket-extremum-per-phase-v1')
        self.assertEqual(len(r['analyticSamples']),123);grid=r['denseCommandGridCertificate']
        self.assertTrue(grid['passed']);self.assertTrue(grid['originalStrictMapUsed']);self.assertEqual(grid['sampleCount'],603)
        self.assertLessEqual(grid['maximumStepS'],.005+1e-14)
        for phase,T in zip(polish.PHASES,path.durations):
            rows=[x for x in r['denseCommandSamples']if x['phase']==phase]
            self.assertEqual(rows[0]['fraction'],0.);self.assertEqual(rows[-1]['fraction'],1.)
            self.assertLessEqual(max((b['fraction']-a['fraction'])*T for a,b in zip(rows,rows[1:])),.005+1e-14)
        extrema=r['extremaVerification'];self.assertTrue(extrema['passed']);self.assertFalse(extrema['wholePathProven'])
        self.assertEqual(len(extrema['phaseMetricCoverage']),3*(2+2*len(self.f.support.equations())))
        self.assertGreater(len(calls),0);self.assertEqual(len(calls),extrema['searchCount'])
        for bounds,method,options in calls:
            self.assertLessEqual(bounds[1]-bounds[0],.01+1e-14);self.assertEqual(method,'bounded')
            self.assertEqual(options,dict(maxiter=40,xatol=1e-8))
        self.assertEqual(r['analyticFailures'],[]);self.assertFalse(r['nativeAdmission'])

    def test_dense_failure_cannot_hide_behind_passing123(self):
        path,r,calls=self.fake_certificate(dense_failure=True);self.assertIsNone(path)
        self.assertTrue(r['twoStageFeasibilityRestoration']['certificateReceipt']['passed'])
        self.assertFalse(r['denseCommandGridCertificate']['passed']);self.assertFalse(r['finiteSampledKinematicPass'])
        self.assertGreater(len(r['analyticFailures']),0);self.assertFalse(r['accepted'])

    def test_nonconverged_time_extremum_is_not_a_certificate(self):
        path,r,calls=self.fake_certificate(scalar_success=False);self.assertIsNone(path)
        self.assertTrue(r['denseCommandGridCertificate']['passed']);self.assertFalse(r['extremaVerification']['passed'])
        self.assertFalse(r['accepted']);self.assertTrue(any(not x['success']for x in r['timeExtremaSearches']))

    def test_unbound_extra_node_or_second_adaptive_loop_rejected(self):
        nodes,binding=self.refinement();normalized=polish.normalized_constraint_nodes(nodes)
        with self.assertRaisesRegex(ValueError,'explicit causal'):polish.refinement_binding(None,normalized,self.knots.reshape(27))
        binding['onlyOneRefinement']=False
        with self.assertRaisesRegex(ValueError,'one-refinement'):polish.refinement_binding(binding,normalized,self.knots.reshape(27))


    def test_envelope_jacobian_835_rows_has_no_per_column_scalar_searches(self):
        nodes,binding=self.refinement();f=self.f
        e=polish.AnalyticPolishEvaluator(f.realizer,f.context,f.limits,f.heading,f.support,self.knots,
            design_only=True,constraint_nodes=nodes,envelope_brackets=binding['sourceBrackets'])
        calls=[]
        def node(path,k,fraction):
            value=float(np.sum(path.independent_knots));block=np.ones(len(e.labels))*(1.+value)
            block[-3]=value+.2+(fraction-.4)**2
            block[46:46+2*len(e.eq)]=value+.3+(fraction-.6)**2
            return block,dict(phase=polish.PHASES[k],fraction=fraction,maximumPoseResidual=0.)
        def scalar(fun,*,bounds,method,options):
            t=sum(bounds)/2.;calls.append(bounds)
            return SimpleNamespace(x=t,fun=fun(t),success=True,nfev=1,nit=1)
        x=self.knots.reshape(27)
        with patch.object(e,'node',side_effect=node),patch.object(polish,'minimize_scalar',side_effect=scalar):
            c,rows=e.evaluate(x);jac=e.constraint_jacobian(x);again=e.constraint_jacobian(x.copy())
        self.assertEqual(c.shape,(835,));self.assertEqual(jac.shape,(835,27));self.assertIs(jac,again)
        self.assertEqual(len(calls),3);self.assertEqual(e.stats['envelopeScalarSearchCalls'],3)
        self.assertEqual(e.stats['fixedPeakJacobianColumns'],27);np.testing.assert_allclose(jac,1.,atol=5e-9)
        self.assertEqual(len(rows),16);self.assertEqual(sum(r.get('envelopeScalar',False)for r in rows),3)
        with self.assertRaises(ValueError):jac[0,0]=99.

    def test_source_bracket_selection_exactly_three_scalar_no_trajectory_calls(self):
        nodes,binding=self.refinement();rows=[]
        for phase in polish.PHASES:
            center=next(b['worstSampleFraction']for b in binding['sourceBrackets']if b['phase']==phase)
            for frac in polish.CERTIFICATE_FRACTIONS:
                rows.append(dict(phase=phase,fraction=frac,accepted=frac!=center,
                    commandAccelerationMps2=1.-(frac-center)**2,supportCaptureMarginM=(frac-center)**2-.01))
        parent=dict(version=polish.VERSION_POLISH,optimizerSuccess=True,accepted=False,analyticSamples=rows,
            analyticFailures=[r for r in rows if not r['accepted']],unadmittedFinalIndependentKnotsRad=self.knots.tolist(),
            finalCandidateSHA256=digest(self.knots.tolist()))
        calls=[]
        def scalar(fun,*,bounds,method,options):
            calls.append(bounds);x=sum(bounds)/2.
            return SimpleNamespace(x=x,fun=fun(x),success=True,nfev=1,nit=1)
        sample=SimpleNamespace(com=Jet([1.,0.,1.],[0.,0.,0.],[.5,0.,0.]),diagnostics=dict(maximumPoseResidual=0.))
        with patch.object(polish,'minimize',side_effect=AssertionError('trajectory forbidden')),\
                patch.object(polish,'minimize_scalar',side_effect=scalar),patch.object(polish.WholeBodyCycle,'sample',return_value=sample):
            added,receipt=polish.select_source_bracket_extrema(self.f.realizer,self.f.context,self.f.limits,self.f.heading,
                SimpleNamespace(equations=lambda:np.array([[1.,0.,0.]])),parent,source_failed_polish_sha256='c'*64,
                design_only=True,entry_elapsed=self.f.context.elapsed)
        self.assertEqual(len(calls),3);self.assertEqual(receipt['scalarSearchCalls'],3);self.assertEqual(receipt['realTrajectoryOptimizerCalls'],0)
        self.assertTrue(receipt['passed']);self.assertEqual(len(added),3);self.assertEqual(receipt['sourceBrackets'],binding['sourceBrackets'])
        for bracket,result,node in zip(receipt['sourceBrackets'],receipt['bracketExtremaResults'],added):
            self.assertLessEqual(bracket['bracketFractions'][0],result['fraction']);self.assertLessEqual(result['fraction'],bracket['bracketFractions'][2])
            self.assertEqual(node['fraction'],result['fraction']);self.assertLess(result['minimumSlack'],0.)


    def test_synthetic_baseline_rejects_before_jacobian_columns_and_shift_failure_raises(self):
        nodes,binding=self.refinement();f=self.f
        e=polish.AnalyticPolishEvaluator(f.realizer,f.context,f.limits,f.heading,f.support,self.knots,
            design_only=True,constraint_nodes=nodes,envelope_brackets=binding['sourceBrackets'])
        with patch.object(e,'node',side_effect=ValueError('fixed Newton branch rejected')),\
                patch.object(e,'fixed_constraints',side_effect=AssertionError('no derivative column permitted')):
            with self.assertRaisesRegex(ValueError,'synthetic analytic baseline'):e.constraint_jacobian(self.knots.reshape(27))
        self.assertEqual(e.stats['fixedPeakJacobianColumns'],0);self.assertEqual(len(e.jacobian_cache),0)
        self.assertTrue(np.all(e.last_result[0]==-1.));self.assertIn('errorType',e.last_result[1][-1])
        # A genuine but negative hard slack is allowed into differentiation;
        # a rejected shifted branch must still escape instead of becoming data.
        rows=tuple(dict(phase=polish.PHASES[k],fraction=v,maximumPoseResidual=0.)for k,v in e.declared_nodes)
        key=self.knots.reshape(27).tobytes();e.envelope_state[key]=(.8,.625,.725)
        with patch.object(e,'evaluate',return_value=(np.full(835,-.01),rows)),\
                patch.object(e,'fixed_constraints',side_effect=ValueError('shifted strict branch rejected')):
            with self.assertRaisesRegex(ValueError,'shifted strict branch'):e.constraint_jacobian(self.knots.reshape(27))
        self.assertEqual(e.stats['fixedPeakJacobianColumns'],0);self.assertEqual(len(e.jacobian_cache),0)


    def test_global_translate_coverage_890_rows_and_closed_endpoints(self):
        nodes,binding=self.refinement();f=self.f
        e=polish.AnalyticPolishEvaluator(f.realizer,f.context,f.limits,f.heading,f.support,self.knots,
            design_only=True,constraint_nodes=nodes,envelope_brackets=binding['sourceBrackets'],global_translate_support=True)
        coverage=e.coverage_receipt();self.assertEqual(len(e.envelope_specs),58);self.assertEqual(len(coverage['constraints']),56)
        expected={(kind,face,a,b)for kind in('COM','capture')for face in range(7)for a,b in zip(polish.FRACTIONS,polish.FRACTIONS[1:])}
        actual={(r['pointKind'],r['faceIndex'],*r['intervalFractions'])for r in coverage['constraints']}
        self.assertEqual(actual,expected);self.assertTrue(coverage['closedIntervalEndpointsCompared'])
        self.assertEqual(coverage['sourceSupportEquationsSHA256'],digest(f.support.equations().tolist()))
        calls=[]
        def node(path,k,fraction):
            value=float(np.sum(path.independent_knots))+fraction;block=np.ones(len(e.labels))*value
            return block,dict(phase=polish.PHASES[k],fraction=fraction,maximumPoseResidual=0.)
        def scalar(fun,*,bounds,method,options):
            t=sum(bounds)/2.;calls.append(bounds)
            return SimpleNamespace(x=t,fun=fun(t),success=True,nfev=1,nit=1)
        with patch.object(e,'node',side_effect=node),patch.object(polish,'minimize_scalar',side_effect=scalar):
            c,rows=e.evaluate(self.knots.reshape(27));jac=e.constraint_jacobian(self.knots.reshape(27))
        self.assertEqual(c.shape,(890,));self.assertEqual(jac.shape,(890,27));self.assertEqual(len(rows),71)
        self.assertEqual(len(calls),58);self.assertEqual(e.stats['fixedPeakJacobianColumns'],27)
        np.testing.assert_allclose(jac,1.,atol=1e-8)
        for row in rows[13:]:
            self.assertTrue(row['closedIntervalEndpointsCompared']);self.assertEqual(row['fraction'],row['sourceIntervalFractions'][0])

    def test_linked_continuation_changes_only_initial_not_objective_anchor(self):
        nodes,binding=self.refinement();initial=self.knots.copy();initial[0,0]+=1e-7
        sha=digest(initial.tolist());anchor=digest(self.knots.tolist())
        history=[dict(parentResultFilename='prior-refinement.json',parentResultSHA256='f'*64,sourceCandidateSHA256=sha,
            initialContinuationCandidateSHA256=sha,objectiveAnchorSHA256=anchor,optimizerCalls=1,hardLimitsUnchanged=True,
            filename='prior-refinement.json',SHA256='f'*64,finalCandidateSHA256=sha)]
        class Stop(RuntimeError):pass
        def harness(fun,x,**kwargs):
            np.testing.assert_array_equal(x,initial.reshape(27));self.assertEqual(fun(self.knots.reshape(27)),0.)
            self.assertGreater(fun(x),0.);np.testing.assert_allclose(kwargs['jac'](x),x-self.knots.reshape(27),atol=0.)
            self.assertIn('jac',kwargs['constraints'][0]);raise Stop('mock continuation only')
        with patch.object(polish.WholeBodyCycle,'sample',side_effect=ValueError('mock certificate rejection')):
            path,r=self.run_mock(harness,constraint_nodes=nodes,collocation_refinement=binding,exchange_history=history,
                initial_knots=initial,global_translate_support_envelopes=True,max_wall_seconds=300.,certificate_max_wall_seconds=180.)
        self.assertIsNone(path);self.assertEqual(r['totalProducerOptimizerCalls'],4);self.assertEqual(r['exchangeHistory'],history)
        self.assertEqual(r['initialContinuationCandidateSHA256'],sha);self.assertEqual(r['objectiveAnchorSHA256'],anchor)
        self.assertEqual(r['activeConstraintRowCounts'],dict(original=832,retainedLiftLandEnvelopes=2,globalTranslateEnvelopes=56,total=890))
        self.assertEqual(len(r['translateFaceEnvelopeCoverage']),56);self.assertEqual(r['totalConstraintRowCount'],890)
        self.assertFalse(r['plannerFastGeometryUsed']);self.assertFalse(r['nativeAdmission'])

    def test_optional_node_geometry_delegates_and_is_cached_without_ownership_change(self):
        nodes,binding=self.refinement();f=self.f
        e=polish.AnalyticPolishEvaluator(f.realizer,f.context,f.limits,f.heading,f.support,self.knots,
            design_only=True,constraint_nodes=nodes)
        path=e.path(self.knots.reshape(27));sample=path.sample('lift',path.durations[0]*.25)
        helper=SimpleNamespace(sample=lambda *a:sample);e.node_geometry=helper
        with patch.object(helper,'sample',return_value=sample)as method:
            a,row=e.node(path,0,.25);b,again=e.node(path,0,.25)
        self.assertEqual(method.call_count,1);self.assertIs(a,b);self.assertIs(row,again)
        np.testing.assert_array_equal(a,e.constraint_block(sample));self.assertIs(path.realizer.arm_map.original,f.arm_map)

if __name__=='__main__':unittest.main(verbosity=2)
