from core.inference import BTGenerator


class _FailSamplingOnce:
    def __init__(self):
        self.calls = []

    def __call__(self, prompt, output_type, **kwargs):
        self.calls.append(kwargs)
        if len(self.calls) == 1:
            raise RuntimeError(
                "probability tensor contains either `inf`, `nan` or element < 0"
            )
        return "<root/>"


def test_probability_failure_retries_with_constrained_greedy_decoding():
    model = _FailSamplingOnce()
    generator = BTGenerator.__new__(BTGenerator)
    generator.model_loaded = True
    generator.outlines_model = model
    generator.xml_pattern = object()

    xml, error = generator.generate_xml("prompt", max_tokens=32, temperature=0.1)

    assert error is None
    assert xml == "<root/>"
    assert model.calls == [
        {"max_new_tokens": 32, "do_sample": True, "temperature": 0.1},
        {"max_new_tokens": 32, "do_sample": False},
    ]


def test_zero_temperature_is_explicitly_greedy():
    model = _FailSamplingOnce()
    generator = BTGenerator.__new__(BTGenerator)
    generator.model_loaded = True
    generator.outlines_model = model
    generator.xml_pattern = object()

    xml, error = generator.generate_xml("prompt", max_tokens=32, temperature=0.0)

    assert xml is None
    assert "probability tensor contains" in error
    assert model.calls == [{"max_new_tokens": 32, "do_sample": False}]
