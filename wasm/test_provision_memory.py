import unittest
from provision_memory import provision, uleb

HEADER = b'\0asm\x01\0\0\0'
MEMORY = b'\x03env\x06memory\x02\x01\x20\x20'


def section(kind, payload):
    return bytes([kind]) + uleb(len(payload)) + payload


class MemoryProvisionTests(unittest.TestCase):
    def test_only_import_limit_changes(self):
        before = section(0, b'\x04testpreserved')
        after = section(0, b'\x04tailunchanged')
        original = HEADER + before + section(2, b'\x01' + MEMORY) + after
        expected = HEADER + before + section(2, b'\x01' + MEMORY[:-1] + uleb(8192)) + after
        self.assertEqual(provision(original), expected)

    def test_minified_release_import(self):
        memory = b'\x01a\x01a\x02\x01\x20\x20'
        result = provision(HEADER + section(2, b'\x01' + memory))
        self.assertEqual(result, HEADER + section(2, b'\x01' + memory[:-1] + uleb(8192)))

    def test_preserves_function_imports(self):
        function = b'\x01m\x01f\x00\x00'
        result = provision(HEADER + section(2, b'\x02' + function + MEMORY))
        self.assertIn(function, result)

    def test_unexpected_layout_rejected(self):
        for payload in [b'\x00', b'\x02' + MEMORY * 2, b'\x01' + MEMORY + b'\x00',
                        b'\x01' + MEMORY[:-1] + b'\x21', b'\x01' + MEMORY[:11] + b'\x03',
                        b'\x01' + MEMORY[:-2] + b'\x00\x00']:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                provision(HEADER + section(2, payload))

    def test_truncated_binary_rejected(self):
        valid = HEADER + section(2, b'\x01' + MEMORY)
        for end in range(len(valid)):
            with self.subTest(end=end), self.assertRaises(ValueError):
                provision(valid[:end])

    def test_duplicate_sections_and_bad_u32_rejected(self):
        for value in [HEADER + section(2, b'\x01' + MEMORY) * 2,
                      HEADER + b'\x02\xff\xff\xff\xff\xff',
                      HEADER + b'\x02\x01\xff']:
            with self.assertRaises(ValueError):
                provision(value)


if __name__ == '__main__':
    unittest.main()
