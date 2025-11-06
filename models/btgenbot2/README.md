# Llama Behavior Tree Generator Model

Fine-tuned Llama 3.2 1B model for generating BehaviorTree.CPP XML from natural language instructions.

## Directory Structure

- `models/` - Fine-tuned model weights and config files (\*.safetensors, tokenizer, configs)
- `schemas/` - outlines-core JSON schemas for structured BT XML generation
- `prompts/` - Prompt templates for different types of BT generation tasks
- `examples/` - Training dataset: instruction→BT XML pairs
- `bt_examples/` - Sample BehaviorTree XML files for Nav2 integration
- `behavior_constraints/` - Runtime safety limits and execution constraints
- `outputs/` - Generated BT outputs (gitignored)

## Structured Generation

Uses `outlines-core` to ensure valid XML output:

- **XML Grammar**: Ensures well-formed BehaviorTree.CPP XML
- **Node Constraints**: Limits to available custom BT nodes
- **Port Validation**: Ensures correct input/output port formats

## Usage

Loaded by `bt_generator_node` (ROS2 node) to convert natural language commands into executable BehaviorTree XML:

```
Natural Language → Llama 3.2-1B + outlines → Valid BT XML → Nav2 Execution
```
