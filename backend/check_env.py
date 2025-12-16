import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def check_environment():
    """Check if all required environment variables are set"""
    required_vars = [
        'QDRANT_URL',
        'QDRANT_API_KEY',
    ]

    # Add database URL - either NEON_DATABASE_URL (legacy) or DATABASE_URL
    database_url = os.getenv('DATABASE_URL')
    neon_database_url = os.getenv('NEON_DATABASE_URL')

    print("Checking environment variables:")
    all_set = True

    for var in required_vars:
        value = os.getenv(var)
        if value:
            print(f"  [OK] {var}: {'Set' if value else 'Not set'}")
        else:
            print(f"  [ERROR] {var}: Missing!")
            all_set = False

    # Check for database URL - either one is acceptable
    if neon_database_url:
        print(f"  [OK] NEON_DATABASE_URL: Set")
    elif database_url:
        print(f"  [OK] DATABASE_URL: Set")
    else:
        print(f"  [ERROR] NEON_DATABASE_URL: Missing!")
        all_set = False

    # Check for API key - either OpenAI or OpenRouter
    openai_api_key = os.getenv('OPENAI_API_KEY')
    openrouter_api_key = os.getenv('OPENROUTER_API_KEY')
    if openai_api_key or openrouter_api_key:
        if openai_api_key:
            print(f"  [OK] OPENAI_API_KEY: Set")
        else:
            print(f"  [OK] OPENROUTER_API_KEY: Set")
    else:
        print(f"  [ERROR] OPENAI_API_KEY: Missing!")
        all_set = False

    # Also check for QDRANT_PORT
    qdrant_port = os.getenv('QDRANT_PORT')
    if qdrant_port:
        print(f"  [OK] QDRANT_PORT: {qdrant_port}")
    else:
        print(f"  [INFO] QDRANT_PORT: Not set (defaulting to 6333)")

    # Check for optional vars
    openai_base = os.getenv('OPENAI_BASE_URL')
    if openai_base:
        print(f"  [OK] OPENAI_BASE_URL: {openai_base}")
    else:
        print(f"  [INFO] OPENAI_BASE_URL: Not set (using default OpenAI)")

    qwen_api_key = os.getenv('QWEN_API_KEY')
    if qwen_api_key:
        print(f"  [OK] QWEN_API_KEY: Set")
    else:
        print(f"  [INFO] QWEN_API_KEY: Not set")

    if all_set:
        print("\n[SUCCESS] All required environment variables are properly set!")
    else:
        print("\n[ERROR] Some required environment variables are missing. Please check your .env file.")

    return all_set

if __name__ == "__main__":
    check_environment()