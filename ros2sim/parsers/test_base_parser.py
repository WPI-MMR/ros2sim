import unittest

from ros2sim.parsers import Parser


class DummyParser(Parser):
  def parse(input_str):
    pass


class TestBasicParser(unittest.TestCase):
  def test_construction(self):
    env = 'test'
    parser = DummyParser(env)
    self.assertEqual(env, parser.env)

if __name__ == '__main__':
  unittest.main()