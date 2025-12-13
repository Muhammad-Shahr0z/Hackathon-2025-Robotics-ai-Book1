# Data Model: Frontend Local Storage Integration for Better Auth

## Authentication Data Entity

**Name**: AuthData
**Description**: Stores user authentication information in browser local storage

**Fields**:
- `token`: string - Session token from Better Auth API
- `userId`: string - Unique user identifier
- `email`: string - User's email address
- `firstName`: string - User's first name
- `lastName`: string - User's last name
- `expiresAt`: string - Token expiration timestamp in ISO format

**Validation Rules**:
- All fields are required when storing
- Token must be a valid JWT or API token format
- Email must be a valid email format
- Expiration time must be in the future when validating

**State Transitions**:
- `Unauthenticated` → `Authenticated` (on successful login)
- `Authenticated` → `Unauthenticated` (on logout or token expiration)

## Local Storage Keys

**Storage Key**: `better_auth_session`
**Format**: JSON string containing AuthData object
**Size Limitation**: Subject to browser local storage limits (typically 5-10MB per origin)

## Relationships
- The AuthData entity corresponds to the user session managed by Better Auth backend
- Data should match the user object returned by `/api/auth/get-session` endpoint