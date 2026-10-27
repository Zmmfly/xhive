#!/usr/bin/env python3
"""Offline stdlib tests for the N32H474 mailbox and capture scripts.

Run with: python3 libs/filters/bench/n32h474/test_scripts.py -v
No J-Link, PXView client, hardware, Xmake or third-party packages are used.
"""
import base64
import binascii
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock


DIRECTORY = Path(__file__).resolve().parent


def load_script(name):
    """Load definitions without executing main or writing project bytecode."""
    path = DIRECTORY / name
    module = types.ModuleType("n32h474_test_" + path.stem)
    module.__file__ = str(path)
    exec(compile(path.read_text(), str(path), "exec"), module.__dict__)
    return module


RUN = load_script("run.py")
CAPTURE = load_script("capture.py")
MAILBOX_ADDRESS = 0x20000130
HEADER_WORDS = 12
CASE_WORDS = 21


def mailbox_words(format_index=0):
    """Encode the frozen C mailbox layout, independently of the decoder."""
    words = [0x51424E43, 1, format_index, 3, 0, 240_000_000,
             0xF00000 if format_index < 2 else 0, 8, 20, 19, 0, 0]
    cycles = [2160, 240, 1920, 480, 1680, 720, 1440, 960, 1200]
    checksum = [0, 0xBFF40000]  # IEEE binary64 -1.25, low word first.
    for nonlinear in (0, 1):
        for n in (2, 4):
            for family, count in ((0, 1), (1, 32), (2, 32), (1, 128), (2, 128)):
                words += [family, nonlinear, n, count, 0, *cycles,
                          128, 64, 256, 32, *checksum, 0x89ABCDEF]
    return words


def packed_mailbox(words):
    return struct.pack(f"<{len(words)}I", *words)


def sample_page(first, count, data, *, truncated=False):
    return {"first_sample": first, "sample_count": count, "byte_count": len(data),
            "bit_order": "lsb0", "bits_per_sample": 1, "truncated": truncated,
            "data": base64.b64encode(data).decode("ascii")}


class OfflineTestCase(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="filters-xhive-script-test-", dir="/tmp")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        # Fail closed if a regression reaches an unmocked command/client path.
        for name in ("run", "check_output"):
            patch = mock.patch.object(subprocess, name, side_effect=AssertionError("external command forbidden"))
            patch.start()
            self.addCleanup(patch.stop)
        no_hardware = types.ModuleType("pxview_automation")
        no_hardware.McpClient = mock.Mock(side_effect=AssertionError("hardware client forbidden"))
        patch = mock.patch.dict(sys.modules, {"pxview_automation": no_hardware})
        patch.start()
        self.addCleanup(patch.stop)


class MailboxTests(OfflineTestCase):
    def test_all_four_formats_and_complete_case_matrix(self):
        expected = {(family, nonlinear, n, count)
                    for nonlinear in (0, 1) for n in (2, 4)
                    for family, count in (("ukf_predict_update", 1),
                                          ("pf_predict_update_estimate", 32),
                                          ("pf_predict_update_estimate", 128),
                                          ("upf_step", 32), ("upf_step", 128))}
        for index, name in enumerate(("f32", "f64", "q15", "q31")):
            with self.subTest(format=name):
                blob = packed_mailbox(mailbox_words(index))
                self.assertEqual(len(blob), 1728)
                result = RUN.decode(blob)
                self.assertEqual(result["format"], name)
                self.assertEqual(len(result["cases"]), 20)
                actual = {(case["family"], case["nonlinear"], case["n"], case["N"])
                          for case in result["cases"]}
                self.assertEqual(actual, expected)
                self.assertEqual(RUN.validate(result), [])

    def test_checksum_status_word_order_and_derived_statistics(self):
        words = mailbox_words()
        words[HEADER_WORDS + 4] = 0xFFFFFFFE
        case = RUN.decode(packed_mailbox(words))["cases"][0]
        self.assertEqual(case["status"], -2)
        self.assertEqual(case["checksum"], -1.25)
        self.assertEqual(case["output_hash"], "89abcdef")
        self.assertEqual(case["m"], 2)
        self.assertEqual(case["persistent_bytes"], 128)
        self.assertEqual(case["descriptor_bytes"], 64)
        self.assertEqual(case["workspace_bytes"], 256)
        self.assertEqual(case["output_bytes"], 32)
        self.assertEqual(case["total_bytes"], 480)
        self.assertEqual(case["median_cycles"], 1200)
        self.assertEqual(case["min_cycles"], 240)
        self.assertEqual(case["max_cycles"], 2160)
        self.assertEqual(case["median_us"], 5.0)

    def test_invalid_mailbox_sizes(self):
        blob = packed_mailbox(mailbox_words())
        for bad in (b"", blob[:-4], blob + b"\0\0\0\0"):
            with self.subTest(size=len(bad)), self.assertRaisesRegex(ValueError, "mailbox size"):
                RUN.decode(bad)

    def test_invalid_abi_dimensions_and_family(self):
        for offset, value, message in ((0, 0, "ABI"), (1, 2, "ABI"),
                                       (2, 4, "dimensions"), (8, 21, "dimensions"),
                                       (HEADER_WORDS, 3, "family")):
            words = mailbox_words()
            words[offset] = value
            with self.subTest(offset=offset), self.assertRaisesRegex(ValueError, message):
                RUN.decode(packed_mailbox(words))

    def test_wrong_clock_is_rejected(self):
        zero_clock = mailbox_words()
        zero_clock[5] = 0
        with self.assertRaisesRegex(ValueError, "clock frequency is zero"):
            RUN.decode(packed_mailbox(zero_clock))
        for clock in (1, 8_000_000, 239_999_999):
            words = mailbox_words()
            words[5] = clock
            with self.subTest(clock=clock):
                self.assertIn("clock readback differs from the fixed 240 MHz configuration",
                              RUN.validate(RUN.decode(packed_mailbox(words))))

    def test_wrong_or_partial_cpacr_is_rejected_for_each_format(self):
        for format_index in range(4):
            for cpacr in ((0, 0x300000) if format_index < 2 else (0xF00000, 0x300000)):
                words = mailbox_words(format_index)
                words[6] = cpacr
                with self.subTest(format=format_index, cpacr=cpacr):
                    self.assertIn("CPACR does not match requested floating-point mode",
                                  RUN.validate(RUN.decode(packed_mailbox(words))))

    def test_incomplete_runs_and_processor_faults_are_rejected(self):
        for offset, value, message in ((3, 2, "run did not complete all 20 cases"),
                                       (3, 4, "run did not complete all 20 cases"),
                                       (8, 19, "run did not complete all 20 cases"),
                                       (10, 1 << 19, "processor fault recorded"),
                                       (11, 1 << 30, "processor fault recorded")):
            words = mailbox_words()
            words[offset] = value
            with self.subTest(offset=offset, value=value):
                self.assertIn(message, RUN.validate(RUN.decode(packed_mailbox(words))))

    def test_duplicate_case_cannot_replace_a_required_case(self):
        words = mailbox_words()
        words[HEADER_WORDS + CASE_WORDS:HEADER_WORDS + 2 * CASE_WORDS] = words[HEADER_WORDS:HEADER_WORDS + CASE_WORDS]
        self.assertIn("case keys differ from the complete 20-case matrix",
                      RUN.validate(RUN.decode(packed_mailbox(words))))

    def test_wrong_case_keys_are_rejected(self):
        for field, value in ((1, 2), (2, 3), (3, 64)):
            words = mailbox_words()
            words[HEADER_WORDS + field] = value
            with self.subTest(field=field):
                self.assertIn("case keys differ from the complete 20-case matrix",
                              RUN.validate(RUN.decode(packed_mailbox(words))))

    def test_nonzero_status_nonfinite_checksum_and_zero_cycles_are_rejected(self):
        for field, value in ((4, 1), (4, 0xFFFFFFFF), (19, 0x7FF80000),
                             (19, 0x7FF00000), (5, 0), (13, 0)):
            words = mailbox_words()
            words[HEADER_WORDS + field] = value
            with self.subTest(field=field, value=value):
                failures = RUN.validate(RUN.decode(packed_mailbox(words)))
                self.assertTrue(any(message.startswith("invalid case ") for message in failures))


class JLinkProtocolTests(OfflineTestCase):
    def test_word_parser_accepts_multiline_words_and_ignores_console_text(self):
        text = ("SEGGER J-Link Commander\nJ-Link>mem32 0x20000130 0x4\n"
                "20000130 = 51424e43 00000001\n"
                "20000138 = 00000003 89ABCDEF  \n"
                "e000ed88 = 00f00000\nPC = 08000218\n"
                "20000140 = not-a-word\n")
        self.assertEqual(RUN.parse_words(text), {0x20000130: 0x51424E43, 0x20000134: 1,
                                               0x20000138: 3, 0x2000013C: 0x89ABCDEF,
                                               0xE000ED88: 0xF00000})
        self.assertEqual(RUN.parse_words("ERROR: Cannot read memory\n"), {})

    def test_read_uses_explicit_hex_count_and_address_order(self):
        probe = RUN.Probe(Path("unused-jlink"), self.directory)
        values = list(range(12))
        response = "20000130 = " + " ".join(f"{value:08X}" for value in values)
        with mock.patch.object(probe, "execute", return_value=response) as execute:
            self.assertEqual(probe.read(MAILBOX_ADDRESS, 12), values)
        execute.assert_called_once_with(["mem32 0x20000130 0xC"], "read")

    def test_missing_word_is_not_silently_filled(self):
        probe = RUN.Probe(Path("unused-jlink"), self.directory)
        with mock.patch.object(probe, "execute", return_value="20000130 = 00000001"), self.assertRaises(KeyError):
            probe.read(MAILBOX_ADDRESS, 2)

    def test_start_rearms_dwt_before_go_and_has_no_swd_reads(self):
        probe = RUN.Probe(Path("unused-jlink"), self.directory)
        with mock.patch.object(probe, "execute", return_value="started") as execute:
            self.assertEqual(probe.start(MAILBOX_ADDRESS, 1.5), "started")
        execute.assert_called_once_with(["h", "w4 0xE000EDFC 0x01000000", "w4 0xE0001000 0",
                                         "w4 0xE0001004 0", "w4 0xE0001000 1",
                                         "w4 0x20000140 1", "g", "sleep 1500"], "start")

    def test_start_rejects_unbounded_or_empty_quiet_window(self):
        probe = RUN.Probe(Path("unused-jlink"), self.directory)
        with mock.patch.object(probe, "execute") as execute:
            for seconds in (-1, 0, 24.001, float("inf"), float("nan")):
                with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                    probe.start(MAILBOX_ADDRESS, seconds)
            execute.assert_not_called()
            probe.start(MAILBOX_ADDRESS, 24)
            self.assertEqual(execute.call_args.args[0][-1], "sleep 24000")


class PulseTests(OfflineTestCase):
    def test_lsb0_byte_boundary_and_unclosed_pulse(self):
        for data, expected in ((b"", []), (b"\0\0", []), (b"\xff", []),
                               (b"\x01", [(0, 1)]), (b"\x80\x01", [(7, 9)]),
                               (b"\x81\x01", [(0, 1), (7, 9)]),
                               (b"\xff\x00\x80", [(0, 8)])):
            with self.subTest(data=data):
                self.assertEqual(list(CAPTURE.pulse_edges(data)), expected)

    def test_packed_decoder_matches_independent_per_sample_reference(self):
        random_source = random.Random(0x51F15)
        payloads = [bytes([value]) for value in range(256)]
        payloads += [bytes(random_source.randrange(256) for _ in range(length)) for length in range(65)]
        for data in payloads:
            expected, rise = [], None
            for position in range(len(data) * 8):
                high = (data[position // 8] >> (position % 8)) & 1
                if high and rise is None:
                    rise = position
                elif not high and rise is not None:
                    expected.append((rise, position))
                    rise = None
            with self.subTest(data=data.hex()):
                self.assertEqual(list(CAPTURE.pulse_edges(data)), expected)


class PackedSampleTests(OfflineTestCase):
    def test_truncated_snapshot_pages_continue_at_actual_count(self):
        client = mock.Mock(spec=["get_samples_meta"])
        client.get_samples_meta.side_effect = [sample_page(0, 16, b"\x80\x01", truncated=True),
                                              sample_page(16, 8, b"\x0f", truncated=True),
                                              sample_page(24, 24, b"\x00\x80\x01")]
        destination = self.directory / "packed.bin"
        result = CAPTURE.read_packed(client, 7, 48, destination)
        self.assertEqual(result, {"samples": 48, "pages": 3, "bytes": 6})
        self.assertEqual(client.get_samples_meta.call_args_list,
                         [mock.call(7, "logic", 0, 47), mock.call(7, "logic", 16, 47),
                          mock.call(7, "logic", 24, 47)])
        self.assertEqual(destination.read_bytes(), b"\x80\x01\x0f\x00\x80\x01")
        self.assertEqual(list(CAPTURE.pulse_edges(destination.read_bytes())), [(7, 9), (16, 20), (39, 41)])

    def test_pages_past_16m_snapshot_and_partial_final_byte(self):
        window = 8 * 1024 * 1024
        block = b"\0" * (window // 8)
        client = mock.Mock(spec=["get_samples_meta"])
        client.get_samples_meta.side_effect = [sample_page(0, window, block),
                                              sample_page(window, window, block),
                                              sample_page(2 * window, 9, b"\x01\x00")]
        destination = self.directory / "over-16m.bin"
        result = CAPTURE.read_packed(client, 11, 2 * window + 9, destination)
        self.assertEqual(result, {"samples": 2 * window + 9, "pages": 3, "bytes": 2 * len(block) + 2})
        self.assertEqual(client.get_samples_meta.call_args_list,
                         [mock.call(11, "logic", 0, window - 1),
                          mock.call(11, "logic", window, 2 * window - 1),
                          mock.call(11, "logic", 2 * window, 2 * window + 8)])
        self.assertEqual(destination.read_bytes(), block + block + b"\x01\x00")
        self.assertEqual(list(CAPTURE.pulse_edges(destination.read_bytes())), [(2 * window, 2 * window + 1)])

    def test_final_unaligned_page_preserves_lsb0_bytes(self):
        client = mock.Mock(spec=["get_samples_meta"])
        client.get_samples_meta.return_value = sample_page(0, 11, b"\x96\x05")
        destination = self.directory / "tail.bin"
        self.assertEqual(CAPTURE.read_packed(client, 0, 11, destination),
                         {"samples": 11, "pages": 1, "bytes": 2})
        self.assertEqual(destination.read_bytes(), b"\x96\x05")
        client.get_samples_meta.assert_called_once_with(0, "logic", 0, 10)

    def test_empty_capture_does_not_request_a_page(self):
        client = mock.Mock(spec=["get_samples_meta"])
        destination = self.directory / "empty.bin"
        self.assertEqual(CAPTURE.read_packed(client, 0, 0, destination),
                         {"samples": 0, "pages": 0, "bytes": 0})
        self.assertEqual(destination.read_bytes(), b"")
        client.get_samples_meta.assert_not_called()

    def test_invalid_page_metadata_is_rejected_without_retry(self):
        changes = ({"first_sample": 8}, {"sample_count": 0}, {"sample_count": -1},
                   {"sample_count": 17}, {"bit_order": "msb0"}, {"bits_per_sample": 8},
                   {"data": base64.b64encode(b"\x01").decode("ascii")})
        for change in changes:
            client = mock.Mock(spec=["get_samples_meta"])
            page = sample_page(0, 16, b"\x01\x00")
            page.update(change)
            client.get_samples_meta.return_value = page
            with self.subTest(change=change), self.assertRaisesRegex(RuntimeError, "invalid packed sample page"):
                CAPTURE.read_packed(client, 2, 16, self.directory / "invalid.bin")
            client.get_samples_meta.assert_called_once_with(2, "logic", 0, 15)

    def test_nonfinal_unaligned_snapshot_is_rejected(self):
        client = mock.Mock(spec=["get_samples_meta"])
        client.get_samples_meta.return_value = sample_page(0, 7, b"\x01", truncated=True)
        with self.assertRaisesRegex(RuntimeError, "non-final snapshot page is not byte-aligned"):
            CAPTURE.read_packed(client, 2, 16, self.directory / "unaligned.bin")
        self.assertEqual(client.get_samples_meta.call_count, 1)

    def test_second_page_must_continue_without_overlap(self):
        client = mock.Mock(spec=["get_samples_meta"])
        client.get_samples_meta.side_effect = [sample_page(0, 8, b"\x01", truncated=True),
                                              sample_page(0, 8, b"\x02")]
        destination = self.directory / "overlap.bin"
        with self.assertRaisesRegex(RuntimeError, "invalid packed sample page"):
            CAPTURE.read_packed(client, 3, 16, destination)
        self.assertEqual(destination.read_bytes(), b"\x01")
        self.assertEqual(client.get_samples_meta.call_count, 2)

    def test_malformed_base64_and_transport_failure_do_not_succeed(self):
        client = mock.Mock(spec=["get_samples_meta"])
        page = sample_page(0, 8, b"\x01")
        page["data"] = "***not-base64***"
        client.get_samples_meta.return_value = page
        with self.assertRaises(binascii.Error):
            CAPTURE.read_packed(client, 1, 8, self.directory / "bad-base64.bin")
        client.get_samples_meta.side_effect = [sample_page(0, 8, b"\x01", truncated=True),
                                              RuntimeError("snapshot unavailable")]
        with self.assertRaisesRegex(RuntimeError, "snapshot unavailable"):
            CAPTURE.read_packed(client, 1, 16, self.directory / "incomplete.bin")


class MainRejectionTests(OfflineTestCase):
    def setUp(self):
        super().setUp()
        self.elf = self.directory / "fixture.elf"
        self.elf.write_bytes(b"offline fixture; never executed")
        self.output = self.directory / "results"
        self.output.mkdir()

    def arguments(self, *extra):
        return ["offline-script", str(self.elf), "--jlink", "unused-jlink",
                "--output", str(self.output), *extra]

    def test_initial_running_phase_is_rejected_before_start_or_second_read(self):
        for extra in ((), ("--start",), ("--prepare-only",)):
            probe = mock.Mock(spec=RUN.Probe)
            header = mailbox_words()[:HEADER_WORDS]
            header[3] = 2
            probe.read.side_effect = [header]
            with self.subTest(arguments=extra), mock.patch.object(sys, "argv", self.arguments(*extra)), \
                 mock.patch.object(RUN, "symbol", return_value=(MAILBOX_ADDRESS, 1728)), \
                 mock.patch.object(RUN, "Probe", return_value=probe), \
                 self.assertRaisesRegex(RuntimeError, "initial SWD read reached a running transaction"):
                RUN.main()
            probe.read.assert_called_once_with(MAILBOX_ADDRESS, HEADER_WORDS)
            probe.start.assert_not_called()
            probe.execute.assert_not_called()
            self.assertEqual(list(self.output.iterdir()), [])

    def test_still_running_after_quiet_window_is_not_polled_again(self):
        probe = mock.Mock(spec=RUN.Probe)
        ready = mailbox_words()[:HEADER_WORDS]
        ready[3] = 1
        running = ready.copy()
        running[3] = 2
        probe.read.side_effect = [ready, running]
        with mock.patch.object(sys, "argv", self.arguments("--start", "--quiet-seconds", "16")), \
             mock.patch.object(RUN, "symbol", return_value=(MAILBOX_ADDRESS, 1728)), \
             mock.patch.object(RUN, "Probe", return_value=probe), \
             self.assertRaisesRegex(RuntimeError, "measurement rejected: SWD observed phase=2"):
            RUN.main()
        self.assertEqual(probe.read.call_count, 2)
        probe.start.assert_called_once_with(MAILBOX_ADDRESS, 16)
        probe.execute.assert_not_called()

    def test_run_nonempty_output_is_rejected_before_probe_or_symbol_lookup(self):
        evidence = self.output / "previous.json"
        evidence.write_bytes(b"preserve previous evidence")
        with mock.patch.object(sys, "argv", self.arguments("--flash")), \
             mock.patch.object(RUN, "symbol") as symbol, mock.patch.object(RUN, "Probe") as probe, \
             self.assertRaisesRegex(ValueError, "run output directory must be empty"):
            RUN.main()
        symbol.assert_not_called()
        probe.assert_not_called()
        self.assertEqual(evidence.read_bytes(), b"preserve previous evidence")

    def test_capture_nonempty_output_is_rejected_before_importing_runner(self):
        evidence = self.output / "previous.bin"
        evidence.write_bytes(b"preserve previous evidence")
        with mock.patch.object(sys, "argv", self.arguments()), \
             mock.patch.object(CAPTURE.importlib.util, "spec_from_file_location") as load, \
             self.assertRaisesRegex(ValueError, "capture output directory must be empty"):
            CAPTURE.main()
        load.assert_not_called()
        self.assertEqual(evidence.read_bytes(), b"preserve previous evidence")

    def test_capture_running_phase_is_rejected_before_client_import(self):
        probe = mock.Mock(spec=RUN.Probe)
        header = mailbox_words()[:HEADER_WORDS]
        header[3] = 2
        probe.read.side_effect = [header]
        runner = types.SimpleNamespace(HEADER_WORDS=12, Probe=mock.Mock(return_value=probe),
                                       symbol=mock.Mock(return_value=(MAILBOX_ADDRESS, 1728)))
        spec = types.SimpleNamespace(loader=types.SimpleNamespace(exec_module=mock.Mock()))
        with mock.patch.object(sys, "argv", self.arguments()), \
             mock.patch.object(CAPTURE.importlib.util, "spec_from_file_location", return_value=spec), \
             mock.patch.object(CAPTURE.importlib.util, "module_from_spec", return_value=runner), \
             self.assertRaisesRegex(RuntimeError, "board not ready"):
            CAPTURE.main()
        probe.read.assert_called_once_with(MAILBOX_ADDRESS, HEADER_WORDS)
        probe.start.assert_not_called()
        probe.execute.assert_not_called()
        sys.modules["pxview_automation"].McpClient.assert_not_called()


if __name__ == "__main__":
    unittest.main()
