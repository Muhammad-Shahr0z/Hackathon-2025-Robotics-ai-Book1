"""
Better Auth implementation for the chatbot-agent backend.
This file implements Better Auth with MCP server integration for both authentication and database operations.
"""
import os
from typing import Optional, Dict, Any
from fastapi import FastAPI, Request, HTTPException, Body
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from dotenv import load_dotenv


# Import our new service and model files
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from services.auth_service import AuthService
from services.validation import ValidationService

# Load environment variables
# load_dotenv()

class BetterAuth:
    """
    Better Auth implementation with MCP server integration
    """

    def _serialize_datetime(self, dt):
        """Helper method to serialize datetime objects"""
        if dt:
            # Check if it's already a string
            if isinstance(dt, str):
                return dt
            # Check if it's a datetime object
            elif hasattr(dt, 'isoformat'):
                return dt.isoformat()
        return dt

    def __init__(self):
        self.auth_service = AuthService()
        self.validation_service = ValidationService()

    async def init_db(self):
        """Initialize the database connection through auth service"""
        await self.auth_service.init_db()

    async def create_user(self, first_name: str, last_name: str, email: str, password: str) -> Dict[str, Any]:
        """Create a new user with first name, last name, email and password"""
        # Validate the input data
        validation_result = self.validation_service.validate_user_registration_data(
            first_name, last_name, email, password
        )

        if not validation_result['valid']:
            raise ValueError(f"Validation failed: {validation_result['errors']}")

        # Create user via auth service
        user = await self.auth_service.create_user(first_name, last_name, email, password)

        return {
            'id': user.id,
            'first_name': user.first_name,
            'last_name': user.last_name,
            'email': user.email,
            'email_verified': user.email_verified,
            'created_at': self._serialize_datetime(user.created_at),
            'active': user.active
        }

    async def get_user_by_email(self, email: str) -> Optional[Dict[str, Any]]:
        """Get user by email"""
        user = await self.auth_service.get_user_by_email(email)

        if not user:
            return None

        return {
            'id': user.id,
            'first_name': user.first_name,
            'last_name': user.last_name,
            'email': user.email,
            'email_verified': user.email_verified,
            'created_at': self._serialize_datetime(user.created_at),
            'updated_at': self._serialize_datetime(user.updated_at),
            'active': user.active
        }

    async def get_user_by_id(self, user_id: str) -> Optional[Dict[str, Any]]:
        """Get user by ID"""
        user = await self.auth_service.get_user_by_id(user_id)

        if not user:
            return None

        return {
            'id': user.id,
            'first_name': user.first_name,
            'last_name': user.last_name,
            'email': user.email,
            'email_verified': user.email_verified,
            'created_at': self._serialize_datetime(user.created_at),
            'updated_at': self._serialize_datetime(user.updated_at),
            'active': user.active
        }

    async def create_session(self, user_id: str) -> Dict[str, Any]:
        """Create a new session for the user"""
        session = await self.auth_service.create_session(user_id)

        return {
            'id': session.id,
            'user_id': session.user_id,
            'token': session.session_token,
            'expires_at': self._serialize_datetime(session.expires_at)
        }

    async def get_session_by_token(self, token: str) -> Optional[Dict[str, Any]]:
        """Get session by token"""
        session = await self.auth_service.get_session_by_token(token)

        if not session:
            return None

        # Get user info as well
        user = await self.auth_service.get_user_by_id(session.user_id)

        return {
            'id': session.id,
            'user_id': session.user_id,
            'token': session.session_token,
            'expires_at': self._serialize_datetime(session.expires_at),
            'user': {
                'id': user.id if user else None,
                'email': user.email if user else None,
                'first_name': user.first_name if user else None,
                'last_name': user.last_name if user else None
            }
        }

    async def delete_session(self, token: str) -> bool:
        """Delete a session by token"""
        return await self.auth_service.delete_session(token)

    async def request_password_reset(self, email: str) -> bool:
        """Request password reset for user"""
        return await self.auth_service.request_password_reset(email)

    async def reset_password(self, token: str, new_password: str) -> bool:
        """Reset user password with token"""
        # Validate the input data
        validation_result = self.validation_service.validate_password_reset_data(
            token, new_password
        )

        if not validation_result['valid']:
            raise ValueError(f"Validation failed: {validation_result['errors']}")

        return await self.auth_service.reset_password(token, new_password)


# Initialize Better Auth instance
# We'll create a new instance per request in serverless environment
# to avoid connection pool issues
def get_auth():
    """Get auth instance for serverless environments"""
    return BetterAuth()


# FastAPI routes for Better Auth
def add_auth_routes(app: FastAPI):
    """Add Better Auth routes to FastAPI app"""

    # Request body models so OpenAPI/Swagger shows payload fields
    class SignUpRequest(BaseModel):
        email: str
        password: str
        name: Optional[str] = None

    class SignInRequest(BaseModel):
        email: str
        password: str

    class RequestPasswordResetRequest(BaseModel):
        email: str

    class ResetPasswordRequest(BaseModel):
        token: str
        newPassword: str = Field(..., alias="newPassword")

    @app.post("/api/auth/sign-up/email")
    async def signup_email(payload: SignUpRequest = Body(...)):
        """Sign up with email and password"""
        try:
            # Initialize auth for this request
            auth = get_auth()
            await auth.init_db()

            email = payload.email
            password = payload.password
            name = payload.name or ''

            # For better compliance with our data model, we should split the name
            # In a real implementation, the frontend would send first_name and last_name separately
            name_parts = name.split(' ', 1) if name else ['', '']
            first_name = name_parts[0] if len(name_parts) > 0 else ''
            last_name = name_parts[1] if len(name_parts) > 1 else ''

            if not email or not password:
                raise HTTPException(status_code=400, detail="Email and password are required")

            # Check if user already exists
            existing_user = await auth.get_user_by_email(email)
            if existing_user:
                raise HTTPException(status_code=400, detail="User already exists")

            # Create new user with first name and last name
            user = await auth.create_user(first_name, last_name, email, password)

            # Create session
            session = await auth.create_session(user['id'])

            return JSONResponse({
                'user': {
                    'id': user['id'],
                    'email': user['email'],
                    'first_name': user['first_name'],
                    'last_name': user['last_name'],
                    # Only return essential fields to match frontend expectations
                },
                'session': {
                    'token': session['token'],
                    'expires_at': auth._serialize_datetime(session['expires_at'])
                }
            })
        except HTTPException:
            raise
        except ValueError as e:
            # Handle validation errors specifically
            raise HTTPException(status_code=400, detail=str(e))
        except Exception as e:
            print(f"Signup error: {e}")
            raise HTTPException(status_code=500, detail="Internal server error")

    @app.post("/api/auth/sign-in/email")
    async def signin_email(payload: SignInRequest = Body(...)):
        """Sign in with email and password"""
        try:
            # Initialize auth for this request
            auth = get_auth()
            await auth.init_db()

            email = payload.email
            password = payload.password

            if not email or not password:
                raise HTTPException(status_code=400, detail="Email and password are required")

            # Get user by email
            user = await auth.get_user_by_email(email)
            if not user:
                raise HTTPException(status_code=401, detail="Invalid email or password")

            # Verify password using auth service
            user_obj = await auth.auth_service.get_user_by_email(email)
            if not user_obj or not await auth.auth_service.verify_password(password, user_obj.password_hash):
                raise HTTPException(status_code=401, detail="Invalid email or password")

            # Create session
            session = await auth.create_session(user['id'])

            return JSONResponse({
                'user': {
                    'id': user['id'],
                    'email': user['email'],
                    'first_name': user['first_name'],
                    'last_name': user['last_name'],
                },
                'session': {
                    'token': session['token'],
                    'expires_at': auth._serialize_datetime(session['expires_at'])
                }
            })
        except HTTPException:
            raise
        except ValueError as e:
            # Handle validation errors specifically
            raise HTTPException(status_code=400, detail=str(e))
        except Exception as e:
            print(f"Signin error: {e}")
            raise HTTPException(status_code=500, detail="Internal server error")

    @app.post("/api/auth/sign-out-clear")
    async def signout_clear(request: Request):
        """Sign out by clearing session"""
        try:
            # Initialize auth for this request
            auth = get_auth()
            await auth.init_db()

            # Get token from authorization header (frontend sends it in the Authorization header)
            auth_header = request.headers.get('Authorization', '')
            token = None
            if auth_header.startswith('Bearer '):
                token = auth_header.replace('Bearer ', '')
            elif auth_header:
                token = auth_header

            if token:
                await auth.delete_session(token)

            response = JSONResponse({"message": "Signed out successfully"})
            return response
        except Exception as e:
            print(f"Signout error: {e}")
            raise HTTPException(status_code=500, detail="Internal server error")

    @app.get("/api/auth/get-session")
    async def get_session(request: Request):
        """Get current session"""
        try:
            # Initialize auth for this request
            auth = get_auth()
            await auth.init_db()

            # Get token from authorization header
            auth_header = request.headers.get('Authorization', '')
            token = None
            if auth_header.startswith('Bearer '):
                token = auth_header.replace('Bearer ', '')
            elif auth_header:
                token = auth_header

            if not token:
                return JSONResponse({"user": None})

            session = await auth.get_session_by_token(token)
            if not session:
                return JSONResponse({"user": None})

            return JSONResponse({
                "user": {
                    "id": session['user']['id'],
                    "email": session['user']['email'],
                    "first_name": session['user']['first_name'],
                    "last_name": session['user']['last_name']
                }
            })
        except Exception as e:
            print(f"Get session error: {e}")
            return JSONResponse({"user": None})

    @app.post("/api/auth/request-password-reset")
    async def request_password_reset(payload: RequestPasswordResetRequest = Body(...)):
        """Request password reset"""
        try:
            # Initialize auth for this request
            auth = get_auth()
            await auth.init_db()

            email = payload.email

            if not email:
                raise HTTPException(status_code=400, detail="Email is required")

            success = await auth.request_password_reset(email)

            return JSONResponse({
                "success": success,
                "message": "If an account exists with this email, a password reset link has been sent."
            })
        except Exception as e:
            print(f"Request password reset error: {e}")
            raise HTTPException(status_code=500, detail="Internal server error")

    @app.post("/api/auth/reset-password")
    async def reset_password(payload: ResetPasswordRequest = Body(...)):
        """Reset password with token"""
        try:
            # Initialize auth for this request
            auth = get_auth()
            await auth.init_db()

            token = payload.token
            # pydantic alias ensures incoming field `newPassword` maps here
            new_password = payload.newPassword

            if not token or not new_password:
                raise HTTPException(status_code=400, detail="Token and new password are required")

            success = await auth.reset_password(token, new_password)

            if not success:
                raise HTTPException(status_code=400, detail="Invalid or expired token")

            return JSONResponse({
                "success": True,
                "message": "Password reset successfully"
            })
        except HTTPException:
            raise
        except Exception as e:
            print(f"Reset password error: {e}")
            raise HTTPException(status_code=500, detail="Internal server error")


# Initialize the auth system
async def initialize_auth():
    """Initialize the auth system"""
    auth = BetterAuth()
    await auth.init_db()
    return auth

