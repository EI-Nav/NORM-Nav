# NaviBot Instruction Parsing

Behavior instruction parsing module that converts natural-language navigation commands into a structured representation for use by the multi-layer costmap system.

## Features

- **Natural language processing**: Uses a large language model (LLM) to parse behavioral instructions in natural language
- **Structured output**: Converts instructions into a standardized constraint format
- **Conflict resolution**: Online constraints take precedence over offline constraints
- **Multi-LLM support**: OpenAI and local LLM backends
- **ROS 2 integration**: Service and topic interfaces

## Architecture

```
Input: online constraints + offline constraints
  ↓
Instruction merge
  ↓
LLM processing
  ↓
Instruction parsing
  ↓
Structured output → multi-layer costmap
```

## Dependencies

```bash
# Core
pip install rclpy openai pydantic

# Local LLM (optional)
pip install requests
```

## Usage

### 1. Start the instruction parsing node

```bash
# OpenAI-compatible service (recommended: use env var)
export LLM_API_KEY="<your_api_key>"
ros2 launch navibot_instruction_parsing instruction_parsing.launch.py

# Optional: override key temporarily via ROS parameter
ros2 launch navibot_instruction_parsing instruction_parsing.launch.py \
    --ros-args -p llm.api_key:=<your_api_key>

# Local LLM
ros2 launch navibot_instruction_parsing instruction_parsing.launch.py \
    --ros-args -p llm.interface_type:=local -p llm.base_url:=http://localhost:11434
```

### 2. Start the instruction publisher (terminal input)

```bash
# Run node
ros2 run navibot_instruction_parsing instruction_publisher_node

# With debug logging
ros2 run navibot_instruction_parsing instruction_publisher_node --ros-args -p log_level:=DEBUG
```

After launch, type instructions in the terminal; enter `quit` to exit.

### 3. Send instructions

#### Via topic
```bash
ros2 topic pub /behavioral_instructions std_msgs/String "
data: '{
  \"online_constraints\": [\"Please walk on the right side of the crosswalk\"],
  \"offline_constraints\": [\"Walk quickly while crossing\"]
}'
"
```

#### Via service
```bash
ros2 service call /parse_behavioral_instructions navibot_instruction_parsing/srv/ParseInstructions "
online_constraints: ['Please walk on the right side of the crosswalk']
offline_constraints: ['Walk quickly while crossing']
"
```

### 4. Receive structured output

```bash
ros2 topic echo /behavioral_constraints
```

## Parameters

### LLM
- `llm.interface_type`: LLM backend (`openai` / `local`)
- `llm.api_key`: Optional API key; when empty, reads `LLM_API_KEY` / `OPENAI_API_KEY` from environment
- `llm.model`: Model name
- `llm.max_tokens`: Max tokens
- `llm.temperature`: Sampling temperature

### Node
- `publish_structured_output`: Whether to publish structured output
- `log_level`: Log level

## Structured output format

```json
{
  "constraints": [
    {
      "object_list": "crosswalk",
      "direction_constrain": "right",
      "velocity_constrain": "fast",
      "traversability_constrain": "traversable"
    }
  ],
  "metadata": {
    "total_constraints": 1,
    "objects": ["crosswalk"],
    "timestamp": null
  }
}
```

### Constraint fields

- `object_list`: Target object name
- `direction_constrain`: Direction (`left` / `right` / `center`)
- `velocity_constrain`: Speed (`slow` / `normal` / `fast`)
- `traversability_constrain`: Traversability (`traversable` / `non-traversable`)

## Examples

### Python API
```python
from navibot_instruction_parsing.instruction_parser import BehavioralInstructionParser, InstructionInput
from navibot_instruction_parsing.llm_interface import OpenAIInterface

llm_interface = OpenAIInterface({"base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1", "model": "qwen-flash"})
parser = BehavioralInstructionParser(llm_interface)

instruction_input = InstructionInput(
    online_constraints=["Walk on the right side of the crosswalk"],
    offline_constraints=["Walk quickly while crossing"]
)

result = parser.parse_instructions(instruction_input)
print(result.to_json())
```

### Tests
```bash
python3 test/test_instruction_parsing.py
```

## Development

### Layout
```
navibot_instruction_parsing/
├── navibot_instruction_parsing/
│   ├── instruction_parser.py      # Core parser
│   ├── instruction_parsing_node.py # ROS 2 parsing node
│   ├── instruction_publisher_node.py # ROS 2 publisher node
│   ├── llm_interface.py           # LLM interface
│   └── structured_output.py       # Structured output types
├── launch/
│   └── instruction_parsing.launch.py
├── config/
│   └── instruction_parsing_params.yaml
├── srv/
│   └── ParseInstructions.srv
└── test/
    └── test_instruction_parsing.py
```

### Extending the LLM interface
```python
from navibot_instruction_parsing.llm_interface import LLMInterface

class CustomLLMInterface(LLMInterface):
    def process_instruction(self, prompt: str) -> str:
        # Custom backend
        pass
```

## Troubleshooting

### Common issues

1. **Service not available**: Build the package and ensure interfaces are generated.
2. **LLM API errors**: Check network and ensure `LLM_API_KEY` (or `OPENAI_API_KEY`) is set.
3. **Parse failures**: Validate input format and LLM responses.

### Debug
```bash
ros2 launch navibot_instruction_parsing instruction_parsing.launch.py log_level:=DEBUG

ros2 run navibot_instruction_parsing instruction_publisher_node --ros-args -p log_level:=DEBUG
```

## License

MIT License — see LICENSE.

## Author

Wang Junhui
