#!/usr/bin/env python3
"""
Test script to check database connection and table structure
"""
import os
from dotenv import load_dotenv
import asyncpg
from urllib.parse import urlparse

# Load environment variables
load_dotenv()

# Get database URL from environment
database_url = os.getenv("DATABASE_URL", "")
# Remove quotes if they exist
database_url = database_url.strip().strip("'\"")

if not database_url:
    print("DATABASE_URL is not set in .env file")
    exit(1)

print(f"Connecting to database: {database_url}")

# Parse the database URL
db_url = urlparse(database_url)
db_config = {
    'host': db_url.hostname,
    'port': db_url.port,
    'database': db_url.path[1:],  # Remove leading slash
    'user': db_url.username,
    'password': db_url.password,
    'ssl': 'require'  # Use SSL as required by Neon
}

async def test_connection():
    try:
        # Create connection pool
        pool = await asyncpg.create_pool(
            host=db_config['host'],
            port=db_config['port'],
            database=db_config['database'],
            user=db_config['user'],
            password=db_config['password'],
            ssl=db_config['ssl'],
            min_size=1,
            max_size=2
        )

        print("[OK] Successfully connected to database")

        # Check if users table exists and its structure
        async with pool.acquire() as conn:
            # Check if table exists
            table_exists = await conn.fetchval("""
                SELECT EXISTS (
                    SELECT FROM information_schema.tables
                    WHERE table_schema = 'public'
                    AND table_name = 'users'
                )
            """)

            if table_exists:
                print("[OK] Users table exists")

                # Get table structure
                columns = await conn.fetch("""
                    SELECT column_name, data_type, is_nullable
                    FROM information_schema.columns
                    WHERE table_name = 'users'
                    ORDER BY ordinal_position
                """)

                print("\n[INFO] Users table structure:")
                for col in columns:
                    print(f"  - {col['column_name']}: {col['data_type']} ({'nullable' if col['is_nullable'] == 'YES' else 'not nullable'})")
            else:
                print("[FAIL] Users table does not exist")

        # Close the pool
        await pool.close()

    except Exception as e:
        print(f"[ERROR] Error connecting to database: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    import asyncio
    asyncio.run(test_connection())