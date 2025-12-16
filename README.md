# Physical AI & Humanoid Robotics Learning Platform

An AI-native educational platform for Physical AI & Humanoid Robotics featuring personalized learning, RAG-powered assistance, and multilingual support.

## Features

- **BetterAuth Authentication** - Secure signup/login with background questionnaire
- **Personalized Learning** - Content adapts to your programming experience
- **Urdu Translation** - Full chapter translation with RTL support
- **RAG Chatbot** - AI assistant trained on course content
- **Progress Tracking** - Track your learning journey
- **Interactive Labs** - Hands-on exercises for each module

## Tech Stack

| Layer | Technology |
|-------|------------|
| Frontend | Next.js 14, Tailwind CSS, shadcn/ui |
| Documentation | Docusaurus 3 |
| Backend | FastAPI, Python 3.11+ |
| Database | Neon PostgreSQL |
| Vector DB | Qdrant |
| AI/ML | OpenAI GPT-4, Ada-002 embeddings |
| Auth | BetterAuth |

## Project Structure

```
.
├── frontend/          # Next.js dashboard & auth
├── backend/           # FastAPI server
├── book/              # Docusaurus textbook
├── specs/             # Project specifications
│   ├── sp.constitution.md
│   ├── sp.specs.md
│   ├── sp.plan.md
│   └── sp.tasks.md
└── .github/workflows/ # CI/CD pipelines
```

## Course Modules

1. **The Robotic Nervous System (ROS 2)** - Introduction to ROS 2 fundamentals
2. **The Digital Twin (Gazebo & Unity)** - Simulation environments
3. **The AI-Robot Brain (NVIDIA Isaac)** - AI-powered robotics
4. **Vision-Language-Action (VLA)** - End-to-end robot learning

## Quick Start

### Prerequisites

- Node.js 20+
- Python 3.11+
- Docker (optional)

### Development Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd physical-ai-platform
   ```

2. **Setup Frontend**
   ```bash
   cd frontend
   cp .env.example .env.local
   npm install
   npm run dev
   ```

3. **Setup Backend**
   ```bash
   cd backend
   cp .env.example .env
   python -m venv venv
   source venv/bin/activate  # Windows: venv\Scripts\activate
   pip install -r requirements.txt
   uvicorn app.main:app --reload
   ```

4. **Setup Docusaurus Book**
   ```bash
   cd book
   npm install
   npm start
   ```

### Docker Compose (All Services)

```bash
cp .env.example .env
docker-compose up -d
```

Services will be available at:
- Frontend: http://localhost:3000
- Backend API: http://localhost:8000
- Book: http://localhost:3001
- Qdrant: http://localhost:6333

## Environment Variables

### Frontend (.env.local)
```env
NEXT_PUBLIC_API_URL=http://localhost:8000
BETTER_AUTH_SECRET=your-secret
DATABASE_URL=postgresql://...
```

### Backend (.env)
```env
DATABASE_URL=postgresql://...
QDRANT_URL=http://localhost:6333
OPENAI_API_KEY=sk-...
```

## Deployment

### Frontend (Vercel)
1. Connect your GitHub repository to Vercel
2. Configure environment variables
3. Deploy automatically on push to main

### Backend (Railway)
1. Create a new Railway project
2. Connect your GitHub repository
3. Configure environment variables
4. Deploy from the `backend` directory

### Book (GitHub Pages)
1. Enable GitHub Pages in repository settings
2. Push to main branch
3. GitHub Actions will build and deploy

## API Documentation

Once running, visit:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

MIT License - see LICENSE file for details

## Acknowledgments

- ROS 2 Documentation
- NVIDIA Isaac SDK
- Gazebo Simulation
- OpenAI API
