from pydantic import BaseModel, Field
from typing import Optional, List, Literal


class PersonalizeRequest(BaseModel):
    content: str = Field(..., min_length=1)


class PersonalizeResponse(BaseModel):
    personalized_content: str
    adaptations: List[str]
    cached: bool = False


class TranslateRequest(BaseModel):
    content: str = Field(..., min_length=1)
    target_language: Literal["ur", "en"] = "ur"


class TranslateResponse(BaseModel):
    translated_content: str
    preserved_terms: List[str]
    cached: bool = False


class SummaryResponse(BaseModel):
    summary: str
    key_concepts: List[str]
    learning_objectives: List[str]
