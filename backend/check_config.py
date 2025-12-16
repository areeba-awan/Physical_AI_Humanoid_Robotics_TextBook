import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Check if we're using OpenRouter API
openrouter_api_key = os.getenv("OPENAI_API_KEY")

if openrouter_api_key and "openrouter" in os.getenv("OPENAI_BASE_URL", ""):
    print("Configuration is set up for OpenRouter API.")
    print("Using base URL:", os.getenv("OPENAI_BASE_URL"))
    print("API Key is present: ", "Yes" if openrouter_api_key else "No")
else:
    print("Configuration might not be properly set up for OpenRouter.")
    print("OPENAI_API_KEY:", "Set" if os.getenv("OPENAI_API_KEY") else "Not set")
    print("OPENAI_BASE_URL:", os.getenv("OPENAI_BASE_URL", "Not set"))

print("\nCurrent environment variables:")
print("QDRANT_URL:", os.getenv("QDRANT_URL"))
print("QDRANT_API_KEY: Set" if os.getenv("QDRANT_API_KEY") else "Not set")
print("OPENAI_API_KEY: Set" if os.getenv("OPENAI_API_KEY") else "Not set")
print("OPENAI_BASE_URL:", os.getenv("OPENAI_BASE_URL"))