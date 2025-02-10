"use client";
import styles from './botao-salvar.module.css';

export default function BotaoSalvar({ onClick, isActive }) {
    return (
        <button
            className={`${styles.botaoSalvar} ${isActive ? styles.active : ''}`}
            onClick={onClick}
            disabled={!isActive} // Desativa o botão quando `isActive` é falso
        >
            Salvar
        </button>
    );
}
