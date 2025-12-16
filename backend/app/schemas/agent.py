from pydantic import BaseModel, Field
from typing import Optional, List, Literal, Dict, Any


class SummarizeRequest(BaseModel):
    content: str = Field(..., min_length=1)
    summary_type: Literal["brief", "detailed", "bullets"] = "detailed"


class SummarizeResponse(BaseModel):
    summary: str
    key_points: List[str]
    agent_used: str = "ChapterSummarizerAgent"


class TranslateAgentRequest(BaseModel):
    content: str = Field(..., min_length=1)
    target_language: Literal["ur", "en"] = "ur"
    preserve_terms: List[str] = []


class TranslateAgentResponse(BaseModel):
    translated_content: str
    preserved_terms: List[str]
    agent_used: str = "UrduTranslatorAgent"


class PersonalizeAgentRequest(BaseModel):
    content: str = Field(..., min_length=1)
    chapter_id: str


class PersonalizeAgentResponse(BaseModel):
    personalized_content: str
    adaptations: List[str]
    agent_used: str = "PersonalizationAgent"


class GenerateRequest(BaseModel):
    topic: str = Field(..., min_length=1)
    content_type: Literal["quiz", "exercise", "explanation", "code"]
    difficulty: Literal["beginner", "intermediate", "advanced"] = "intermediate"
    count: int = Field(5, ge=1, le=20)


class GenerateResponse(BaseModel):
    generated_content: Dict[str, Any]
    content_type: str
    agent_used: str = "ContentGeneratorAgent"
