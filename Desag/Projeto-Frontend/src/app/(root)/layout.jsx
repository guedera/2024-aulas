import React from 'react';
import Header from "../(root)/_components/header";

export default function Layout({ children }) {
    return (
        <>
            <Header />
            {children}
        </>
    );
}
