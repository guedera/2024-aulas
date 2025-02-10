/*
 * NÃO MUDE NADA A MENOS QUE TENHA UM BOM MOTIVO.
 * E, mesmo que tenha, confirme com o professor.
 */

import { Inter } from 'next/font/google';
import './globals.css';

const inter = Inter({ subsets: ['latin'] });

export const metadata = {
    title: 'Frontend',
};

export default function RootLayout({ children }) {
    return (
        <html lang="pt-br">
            <body className={inter.className}>{children}</body>
        </html>
    );
}
