import React, { useState, useEffect, JSX } from 'react';
import { useLocation } from '@docusaurus/router';
import { translate } from '@docusaurus/Translate';
import Link from '@docusaurus/Link';
import { logout, getAccessToken } from '@site/src/lib/authAdapter';

interface CustomNavbarItemProps {
  readonly mobile?: boolean;
  readonly position?: 'left' | 'right';
}

export default function CustomAuthNavbarItem({
  mobile,
  position,
}: CustomNavbarItemProps): JSX.Element {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [userEmail, setUserEmail] = useState('');
  const [isLoading, setIsLoading] = useState(true);
  const location = useLocation();

  useEffect(() => {
    // Check if user is logged in by checking for access token
    const token = getAccessToken();
    setIsLoggedIn(!!token);

    // For a complete implementation, we would fetch user details from the backend
    // For now, we'll just extract email from localStorage if available
    if (token) {
      // In a real implementation, you would decode the JWT to get user info
      // or make an API call to get user details
      // For this example, we'll use a placeholder email
      setUserEmail('user@example.com');
    }

    setIsLoading(false);

    // Set up event listener to update state when auth changes
    const handleStorageChange = () => {
      const newToken = getAccessToken();
      setIsLoggedIn(!!newToken);
    };

    window.addEventListener('storage', handleStorageChange);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }, []);

  const handleLogout = () => {
    logout();
    setIsLoggedIn(false);
    // Redirect to login page after logout
    window.location.href = '/login';
  };

  if (mobile) {
    // For mobile menu
    if (isLoggedIn) {
      return (
        <div className="dropdown dropdown--navbar">
          <div className="navbar__link">
            👤 {userEmail.split('@')[0]}
          </div>
          <ul className="dropdown__menu">
            <li>
              <Link className="dropdown__link" to="/profile">
                Your Profile
              </Link>
            </li>
            <li>
              <button
                className="dropdown__link dropdown__link--btn"
                onClick={handleLogout}
              >
                Sign Out
              </button>
            </li>
          </ul>
        </div>
      );
    } else {
      return (
        <Link className="navbar__item navbar__link" to="/login">
          Sign In
        </Link>
      );
    }
  }

  // For desktop navbar
  if (isLoading) {
    return (
      <div className={`navbar__item navbar__link ${position === 'right' ? 'navbar__item--right' : ''}`}>
        Loading...
      </div>
    );
  }

  if (isLoggedIn) {
    return (
      <div className={`navbar__item navbar__link ${position === 'right' ? 'navbar__item--right' : ''}`}>
        <div className="dropdown dropdown--hoverable">
          <a
            href="#"
            className="navbar__link"
          >
            👤 {userEmail.split('@')[0]}
          </a>
          <ul className="dropdown__menu">
            <li>
              <Link className="dropdown__link" to="/profile">
                Your Profile
              </Link>
            </li>
            <li>
              <button
                className="dropdown__link dropdown__link--btn"
                onClick={handleLogout}
              >
                Sign Out
              </button>
            </li>
          </ul>
        </div>
      </div>
    );
  } else {
    return (
      <Link
        className={`navbar__item navbar__link ${position === 'right' ? 'navbar__item--right' : ''}`}
        to="/login"
      >
        Sign In
      </Link>
    );
  }
}