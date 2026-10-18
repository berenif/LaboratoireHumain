"""Pure tests for the opt-in ROM-only adaptation. No original XML is published."""
import unittest
import xml.etree.ElementTree as ET
from source_hip_mobility import adapt_xml, HIP_RANGES_DEG


class HipMobilityRecipe(unittest.TestCase):
    def source(self):
        joints = ''.join(f'<joint name="{side}_hip_{axis}" axis="{axis}" range="{lo:g} {hi:g}" class="unchanged"/>\n'
                         for side in ('right','left') for axis,(lo,hi) in HIP_RANGES_DEG.items())
        return '<mujoco><worldbody><body>\n'+joints+'</body></worldbody><actuator><motor gear="40"/></actuator></mujoco>'

    def test_exact_six_ranges_only(self):
        text = self.source()
        changed, receipt = adapt_xml(text)
        self.assertEqual(len(receipt), 6)
        for row in receipt:
            self.assertEqual(row['afterDeg'], [row['beforeDeg'][0]-10, row['beforeDeg'][1]+10])
        root = ET.fromstring(changed)
        self.assertEqual(root.find('.//motor').attrib, {'gear':'40'})
        self.assertTrue(all(j.attrib['class']=='unchanged' for j in root.findall('.//joint')))

    def test_unknown_range_rejected(self):
        with self.assertRaises(AssertionError):
            adapt_xml(self.source().replace('range="-25 5"','range="-26 5"',1))

    def test_missing_hip_rejected(self):
        with self.assertRaises(AssertionError):
            adapt_xml(self.source().replace('left_hip_y','other'))

    def test_unbounded_expansion_rejected(self):
        with self.assertRaises(AssertionError):
            adapt_xml(self.source(), 100.)

    def test_zero_expansion_semantically_identical(self):
        text, _ = adapt_xml(self.source(), 0.)
        self.assertEqual(ET.tostring(ET.fromstring(text)), ET.tostring(ET.fromstring(self.source())))

if __name__ == '__main__':
    unittest.main()
