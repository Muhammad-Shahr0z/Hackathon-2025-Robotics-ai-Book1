"""
Authentication Service for the Better Auth implementation.
This file implements the service layer for authentication functionality.
"""
import os
import secrets
import hashlib
import datetime
from typing import Optional, Dict, Any
from urllib.parse import urlparse
import bcrypt
import asyncpg
from cryptography.fernet import Fernet
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from models.user import User
from models.session import AuthSession
from models.password_reset import PasswordResetToken


class AuthService:
    """
    Authentication Service implementing Better Auth functionality
    """
    def __init__(self):
        # Get environment variables
        self.better_auth_secret = os.getenv("BETTER_AUTH_SECRET", "0T4nMgkuZ53EDXWXZ2cT6rWz6EBqaRgh")
        self.better_auth_url = os.getenv("BETTER_AUTH_URL", "http://localhost:3000")
        self.database_url = os.getenv("DATABASE_URL", "")


        if not self.database_url:
            raise ValueError("DATABASE_URL is required in .env file")

        # Parse the database URL
        db_url = urlparse(self.database_url)
        self.db_config = {
            'host': db_url.hostname,
            'port': db_url.port,
            'database': db_url.path[1:],  # Remove leading slash
            'user': db_url.username,
            'password': db_url.password,
            'ssl': 'require'  # Use SSL as required by Neon
        }

        self.pool = None
        self.cipher_suite = Fernet(self._get_or_create_key())

    def _get_or_create_key(self):
        """Create a key for encryption/decryption"""
        key = os.getenv("ENCRYPTION_KEY")
        if not key:
            key = Fernet.generate_key().decode()
            # In production, save this to environment
        return key.encode()

    async def init_db(self):
        """Initialize the database connection pool"""
        # For serverless environments, we'll create connections on demand
        # rather than maintaining a persistent pool
        if not self.pool:
            # Create a single connection for this request context
            # In serverless, connection pooling is not efficient
            try:
                self.pool = await asyncpg.create_pool(
                    host=self.db_config['host'],
                    port=self.db_config['port'],
                    database=self.db_config['database'],
                    user=self.db_config['user'],
                    password=self.db_config['password'],
                    ssl=self.db_config['ssl'],
                    min_size=1,
                    max_size=1  # Keep it minimal for serverless
                )

                # Create required tables if they don't exist
                await self._create_tables()
            except Exception as e:
                print(f"Database connection error: {e}")
                raise

    async def _create_tables(self):
        """Create required Better Auth tables"""
        async with self.pool.acquire() as conn:
            # Check if users table exists and create if it doesn't
            table_exists = await conn.fetchval("""
                SELECT EXISTS (
                    SELECT FROM information_schema.tables
                    WHERE table_schema = 'public'
                    AND table_name = 'users'
                )
            """)

            if not table_exists:
                # Create users table with our schema
                await conn.execute("""
                    CREATE TABLE users (
                        id TEXT PRIMARY KEY DEFAULT gen_random_uuid()::text,
                        first_name TEXT,
                        last_name TEXT,
                        email TEXT UNIQUE NOT NULL,
                        password_hash TEXT NOT NULL,
                        email_verified BOOLEAN DEFAULT FALSE,
                        active BOOLEAN DEFAULT TRUE,
                        created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                        updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
                    )
                """)
            else:
                # Check if our required columns exist and add them if missing
                # Check for first_name column
                first_name_exists = await conn.fetchval("""
                    SELECT EXISTS (
                        SELECT FROM information_schema.columns
                        WHERE table_name = 'users' AND column_name = 'first_name'
                    )
                """)

                if not first_name_exists:
                    await conn.execute("ALTER TABLE users ADD COLUMN first_name TEXT")

                # Check for last_name column
                last_name_exists = await conn.fetchval("""
                    SELECT EXISTS (
                        SELECT FROM information_schema.columns
                        WHERE table_name = 'users' AND column_name = 'last_name'
                    )
                """)

                if not last_name_exists:
                    await conn.execute("ALTER TABLE users ADD COLUMN last_name TEXT")

                # Check for password_hash column
                password_hash_exists = await conn.fetchval("""
                    SELECT EXISTS (
                        SELECT FROM information_schema.columns
                        WHERE table_name = 'users' AND column_name = 'password_hash'
                    )
                """)

                if not password_hash_exists:
                    await conn.execute("ALTER TABLE users ADD COLUMN password_hash TEXT NOT NULL DEFAULT ''")

                # Check for active column
                active_exists = await conn.fetchval("""
                    SELECT EXISTS (
                        SELECT FROM information_schema.columns
                        WHERE table_name = 'users' AND column_name = 'active'
                    )
                """)

                if not active_exists:
                    await conn.execute("ALTER TABLE users ADD COLUMN active BOOLEAN DEFAULT TRUE")

                # Check for updated_at column
                updated_at_exists = await conn.fetchval("""
                    SELECT EXISTS (
                        SELECT FROM information_schema.columns
                        WHERE table_name = 'users' AND column_name = 'updated_at'
                    )
                """)

                if not updated_at_exists:
                    await conn.execute("ALTER TABLE users ADD COLUMN updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()")

            # Create sessions table if it doesn't exist
            sessions_table_exists = await conn.fetchval("""
                SELECT EXISTS (
                    SELECT FROM information_schema.tables
                    WHERE table_schema = 'public'
                    AND table_name = 'sessions'
                )
            """)

            if not sessions_table_exists:
                await conn.execute("""
                    CREATE TABLE sessions (
                        id TEXT PRIMARY KEY DEFAULT gen_random_uuid()::text,
                        user_id TEXT NOT NULL REFERENCES users(id) ON DELETE CASCADE,
                        token TEXT UNIQUE NOT NULL,
                        expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
                        created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                        updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
                    )
                """)

            # Create verification table for password reset tokens if it doesn't exist
            verification_table_exists = await conn.fetchval("""
                SELECT EXISTS (
                    SELECT FROM information_schema.tables
                    WHERE table_schema = 'public'
                    AND table_name = 'verification'
                )
            """)

            if not verification_table_exists:
                await conn.execute("""
                    CREATE TABLE verification (
                        id TEXT PRIMARY KEY DEFAULT gen_random_uuid()::text,
                        identifier TEXT NOT NULL,  -- email for password reset
                        value TEXT NOT NULL,  -- token
                        expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
                        used BOOLEAN DEFAULT FALSE,
                        created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
                        updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
                    )
                """)

            # Create indexes for better performance (if they don't exist)
            # Check and create indexes if they don't exist
            try:
                await conn.execute("CREATE INDEX IF NOT EXISTS idx_users_email ON users(email)")
            except:
                pass  # Index might already exist
            try:
                await conn.execute("CREATE INDEX IF NOT EXISTS idx_sessions_token ON sessions(token)")
            except:
                pass
            try:
                await conn.execute("CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON sessions(user_id)")
            except:
                pass
            try:
                await conn.execute("CREATE INDEX IF NOT EXISTS idx_verification_identifier ON verification(identifier)")
            except:
                pass

    async def hash_password(self, password: str) -> str:
        """Hash password using bcrypt"""
        salt = bcrypt.gensalt()
        hashed = bcrypt.hashpw(password.encode('utf-8'), salt)
        return hashed.decode('utf-8')

    async def verify_password(self, password: str, hashed: str) -> bool:
        """Verify password against hash"""
        return bcrypt.checkpw(password.encode('utf-8'), hashed.encode('utf-8'))

    async def create_user(self, first_name: str, last_name: str, email: str, password: str) -> User:
        """Create a new user with first name, last name, email and password"""
        password_hash = await self.hash_password(password)
        created_at = datetime.datetime.now(datetime.timezone.utc)

        async with self.pool.acquire() as conn:
            user_record = await conn.fetchrow("""
                INSERT INTO users (first_name, last_name, email, password_hash, created_at, updated_at)
                VALUES ($1, $2, $3, $4, $5, $6)
                RETURNING id, first_name, last_name, email, password_hash, email_verified, active, created_at, updated_at
            """, first_name, last_name, email, password_hash, created_at, created_at)

            return User.from_db_record(user_record)

    async def get_user_by_email(self, email: str) -> Optional[User]:
        """Get user by email"""
        async with self.pool.acquire() as conn:
            user_record = await conn.fetchrow("""
                SELECT id, first_name, last_name, email, password_hash, email_verified, active, created_at, updated_at
                FROM users
                WHERE email = $1 AND active = TRUE
            """, email)

            if not user_record:
                return None

            return User.from_db_record(user_record)

    async def get_user_by_id(self, user_id: str) -> Optional[User]:
        """Get user by ID"""
        async with self.pool.acquire() as conn:
            user_record = await conn.fetchrow("""
                SELECT id, first_name, last_name, email, password_hash, email_verified, active, created_at, updated_at
                FROM users
                WHERE id = $1 AND active = TRUE
            """, user_id)

            if not user_record:
                return None

            return User.from_db_record(user_record)

    async def create_session(self, user_id: str) -> AuthSession:
        """Create a new session for the user"""
        token = secrets.token_urlsafe(32)
        created_at = datetime.datetime.now(datetime.timezone.utc)
        expires_at = created_at + datetime.timedelta(days=30)

        async with self.pool.acquire() as conn:
            session_record = await conn.fetchrow("""
                INSERT INTO sessions (user_id, token, expires_at, created_at, updated_at)
                VALUES ($1, $2, $3, $4, $5)
                RETURNING id, user_id, token, expires_at, created_at, updated_at
            """, user_id, token, expires_at, created_at, created_at)

            return AuthSession(
                id=session_record['id'],
                user_id=session_record['user_id'],
                session_token=session_record['token'],
                expires_at=session_record['expires_at'],
                created_at=session_record['created_at']
            )

    async def get_session_by_token(self, token: str) -> Optional[AuthSession]:
        """Get session by token"""
        async with self.pool.acquire() as conn:
            session_record = await conn.fetchrow("""
                SELECT s.id, s.user_id, s.token, s.expires_at, s.created_at, s.updated_at
                FROM sessions s
                WHERE s.token = $1 AND s.expires_at > NOW()
            """, token)

            if not session_record:
                return None

            return AuthSession(
                id=session_record['id'],
                user_id=session_record['user_id'],
                session_token=session_record['token'],
                expires_at=session_record['expires_at'],
                created_at=session_record['created_at']
            )

    async def delete_session(self, token: str) -> bool:
        """Delete a session by token"""
        async with self.pool.acquire() as conn:
            result = await conn.execute("""
                DELETE FROM sessions WHERE token = $1
            """, token)

            return result != "DELETE 0"

    async def request_password_reset(self, email: str) -> bool:
        """Request password reset for user"""
        user = await self.get_user_by_email(email)
        if not user:
            # Don't reveal if email exists
            return True

        # Generate reset token
        token = secrets.token_urlsafe(32)
        expires_at = datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(hours=1)

        async with self.pool.acquire() as conn:
            await conn.execute("""
                INSERT INTO verification (identifier, value, expires_at)
                VALUES ($1, $2, $3)
            """, email, token, expires_at)

        # Create reset link
        reset_link = f"{self.better_auth_url}/reset-password?token={token}"

        # Send password reset email
        email_subject = "Password Reset Request"
        email_body = f"""
        You have requested to reset your password.

        Please click the link below to reset your password:
        {reset_link}

        This link will expire in 1 hour.

        If you did not request this, please ignore this email.
        """

        # In a real implementation, we would send an actual email
        # For now, we'll just print the details (in production, use an email service)
        print(f"EMAIL TO: {email}")
        print(f"SUBJECT: {email_subject}")
        print(f"BODY: {email_body}")
        print("---")

        return True

    async def reset_password(self, token: str, new_password: str) -> bool:
        """Reset user password with token"""
        async with self.pool.acquire() as conn:
            # Check if token exists, is not expired, and not used
            verification = await conn.fetchrow("""
                SELECT identifier, used FROM verification
                WHERE value = $1 AND expires_at > NOW() AND used = FALSE
            """, token)

            if not verification or verification['used']:
                return False

            email = verification['identifier']

            # Hash new password
            password_hash = await self.hash_password(new_password)

            # Update password in users table
            result = await conn.execute("""
                UPDATE users
                SET password_hash = $1, updated_at = NOW()
                WHERE email = $2
            """, password_hash, email)

            if result == "UPDATE 0":
                return False

            # Mark verification token as used
            await conn.execute("""
                UPDATE verification
                SET used = TRUE, updated_at = NOW()
                WHERE value = $1
            """, token)

            return True