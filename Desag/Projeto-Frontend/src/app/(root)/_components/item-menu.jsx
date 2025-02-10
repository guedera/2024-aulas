"use client"
import React from 'react';
import styles from './item-menu.module.css';

export default function ItemMenu(props) {
    return (
        <div className={styles.itemMenu}>
            <a href={props.uri} className={styles.link}>
                <p>{props.children}</p>
            </a>
        </div>
    );
}
