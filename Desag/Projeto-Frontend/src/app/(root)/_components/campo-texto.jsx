"use client"
import React from 'react';
import styles from './campo-texto.module.css';

export default function CampoTexto(props) {
    return (
        <div>
            {props.name && (
                <label className={styles.label}>{props.name}</label>
            )}
            <input 
                type="text" 
                placeholder="Digite aqui..." 
                className={styles.input} 
                value={props.value} 
                onChange={(e) => props.onChange(e.target.value)}
            />
        </div>
    );
}
